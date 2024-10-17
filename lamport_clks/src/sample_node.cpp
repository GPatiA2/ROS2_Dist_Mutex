#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <array>

using std::placeholders::_1;
using std::placeholders::_2;

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/srv/string_srv.hpp"

using namespace std::chrono_literals;

// Cool type alias for future types in ROS2
template<class T>
using future_t = typename rclcpp::Client<T>::SharedFuture;

typedef enum {HELD, WANTED, RELEASED} State;

struct timed_request{
  int timestamp;
  int id;
  timed_request(int t, int i);
};
timed_request::timed_request(int t, int i) : timestamp(t), id(i) {}; 

struct{
  bool operator()(const timed_request t1, const timed_request t2) const
  {
    return t1.timestamp > t2.timestamp || (t1.timestamp == t2.timestamp && t1.id > t2.id);
  }
} timed_request_compare;

bool operator>(const timed_request t1, const timed_request t2){
  return timed_request_compare(t1, t2);
}

std::string node_namespace(int num_id){
  return "/node_" + std::to_string(num_id);
}

class DistMutexNode : public rclcpp::Node
{
public:
  DistMutexNode(int total_nodes)
  : Node("minimal_publisher"), count_(0), total_nodes(total_nodes)
  {
    this->declare_parameter("num_id", 0);
    num_id = this->get_parameter("num_id").as_int();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting node with id %d", num_id);

    timer = this->create_wall_timer(
      1500ms, std::bind(&DistMutexNode::timer_callback, this));

    s = RELEASED;

    pending_requests = std::priority_queue<timed_request, std::vector<timed_request>, decltype(timed_request_compare)>();

    pending_replies = std::vector<bool>(total_nodes);

    std::string this_namespace = "/node_" + std::to_string(num_id);

    // To handle requests from others to enter the critical section
    req_sc_enter_service = this->create_service<test_msgs::srv::StringSrv>(this_namespace + "/answer", 
     std::bind(&DistMutexNode::ack_req_enter_callback, this, _1, _2));

    // To handle permission from others to enter the critical section
    allow_sc_enter_service = this->create_service<test_msgs::srv::StringSrv>(this_namespace + "/grant_perm",
      std::bind(&DistMutexNode::gather_permission, this, _1, _2));

    // To request access to the critical section
    sc_request_clients = std::vector<rclcpp::Client<test_msgs::srv::StringSrv>::SharedPtr> ();
    for (int i = 0; i < total_nodes; i++){
      sc_request_clients.push_back(this->create_client<test_msgs::srv::StringSrv>(node_namespace(i) + "/answer"));
    }

    // To grant access to the critical section to others
    sc_grant_clients = std::vector<rclcpp::Client<test_msgs::srv::StringSrv>::SharedPtr> ();
    for (int i = 0; i < total_nodes; i++){
      sc_grant_clients.push_back(this->create_client<test_msgs::srv::StringSrv>(node_namespace(i) + "/grant_perm"));
    }
  }
  

private:
  void timer_callback()   
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "count = %d", count_); 
    count_++;
    if(s == RELEASED){
      s = WANTED;
      ts_request = count_;
      requested = true;
      broadcast_request(); 
      std::fill(pending_replies.begin(), pending_replies.end(), false); 
    }
    else if(s == WANTED){
      int total_responses = std::count(pending_replies.begin(), pending_replies.end(), true);
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received %d permissions", total_responses);
      if(std::count(pending_replies.begin(), pending_replies.end(), true) == total_nodes - 1){
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received all permissions");
        s = HELD;
        std::fill(pending_replies.begin(), pending_replies.end(), false); 
      }
      else{
        return;
      } 
    } 
    if(s == HELD){
      critical_section();
      s = RELEASED;
      requested = false;
      reply_pending_requests();
    }

  }

  void reply_pending_requests(){
    while(!pending_requests.empty()){
      timed_request tr = pending_requests.top();
      pending_requests.pop();
      grant_permission(std::to_string(tr.id), count_);     
    }
  }

  // Store requests from others to enter the critical section
  void ack_req_enter_callback(const std::shared_ptr<test_msgs::srv::StringSrv::Request> request,
    std::shared_ptr<test_msgs::srv::StringSrv::Response> response)
  {
    std::string rid = request->request_id;
    std::string msg = "Received request from drone ";
    msg = msg.append(rid);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), msg.c_str());
    update_count(request->time_request);  
    response->response_id = std::to_string(num_id);
    response->time_response = count_; 

    timed_request tr_other(request->time_request, std::stoi(request->request_id));
    timed_request tr_self(ts_request, num_id);

    if((s == HELD) || (s == WANTED && tr_other > tr_self)){
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adding request to queue since %d > %d", tr_other.timestamp, tr_self.timestamp);
      timed_request tr(request->time_request, std::stoi(request->request_id)); 
      pending_requests.push(tr);
    }
    else{
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Granting permission since %d > %d", tr_self.timestamp, tr_other.timestamp);
      grant_permission(request->request_id, request->time_request);
    }
  }

  // Store authorization from others to enter the critical section
  void gather_permission(const std::shared_ptr<test_msgs::srv::StringSrv::Request> request,
    std::shared_ptr<test_msgs::srv::StringSrv::Response> response)
  {
    update_count(request->time_request);
    pending_replies[std::stoi(request->request_id)] = true;
    response->response_id = std::to_string(num_id);
    response->time_response = count_;
  }

  // Grant permission to another process to enter the critical section
  void grant_permission(const std::string request_id, const int request_time)
  {
    auto permission_msg = std::make_shared<test_msgs::srv::StringSrv::Request>();
    permission_msg->time_request = update_count(request_time);
    permission_msg->request_id = std::to_string(num_id);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Granting permission to node %d", std::stoi(request_id));
    sc_grant_clients[std::stoi(request_id)]->async_send_request(permission_msg);
  }

  void broadcast_request(){
    count_++;
    for (int i = 0; i < total_nodes; i++){
      if(i != num_id){
        auto request = std::make_shared<test_msgs::srv::StringSrv::Request>();
        request->time_request = count_; 
        request->request_id = std::to_string(num_id);
        future_t<test_msgs::srv::StringSrv> result_future = sc_request_clients[i]->async_send_request(request).share(); 
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent request to node %d in time %d", i, count_);
      }
    }
  }

  void critical_section(){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node %d entering critical section", num_id);
    sleep(2);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node %d exiting critical section", num_id);
  }

  int update_count(int req_time){
    count_ = std::max(count_, req_time) + 1;
    return count_;
  } 

  rclcpp::TimerBase::SharedPtr timer;

  // Service to store requests to enter the critical section
  rclcpp::Service<test_msgs::srv::StringSrv>::SharedPtr req_sc_enter_service;

  // Service to be granted access to the critical section to others
  rclcpp::Service<test_msgs::srv::StringSrv>::SharedPtr allow_sc_enter_service;

  // Clients to make requests when entering the critical section 
  std::vector<rclcpp::Client<test_msgs::srv::StringSrv>::SharedPtr>  sc_request_clients;

  // Clients to grant access to the critical section to others
  std::vector<rclcpp::Client<test_msgs::srv::StringSrv>::SharedPtr>  sc_grant_clients;

  // Priority queue to store pending requests to answer
  std::priority_queue<timed_request, std::vector<timed_request>, decltype(timed_request_compare)> pending_requests;
  
  // Vector to store authorizations to enter the critical section 
  std::vector<bool>  pending_replies; 

  int ts_request;
  bool requested;

  int count_;
  int num_id;
  int total_nodes;
  State s;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistMutexNode>(4));
  rclcpp::shutdown();
  return 0;
}
