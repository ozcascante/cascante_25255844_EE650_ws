#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

class Patrol : public plansys2::ActionExecutorClient
{
public:
  Patrol()
  : plansys2::ActionExecutorClient("patrol", 1s)
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    progress_ = 0.0;
    
    // Get room name from action arguments: (patrol ecobot <room>)
    room_ = get_arguments()[1];
    
    RCLCPP_INFO(get_logger(), "Starting patrol of %s", room_.c_str());

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_pub_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    cmd_vel_pub_->on_deactivate();
    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.1;
      send_feedback(progress_, "Patrol running");

      // Spin in place
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = 0.5;
      cmd_vel_pub_->publish(cmd);
      
    } else {
      // Stop
      geometry_msgs::msg::Twist cmd;
      cmd_vel_pub_->publish(cmd);

      // Check if room is unoccupied and light is on
      check_and_turn_off_light();

      RCLCPP_INFO(get_logger(), "Patrol of %s completed", room_.c_str());
      finish(true, 1.0, "Patrol completed");
    }
  }

  void check_and_turn_off_light()
  {
    // Check predicates
    bool is_occupied = problem_expert_->existPredicate(
      plansys2::Predicate("(occupied " + room_ + ")"));
    bool light_is_on = problem_expert_->existPredicate(
      plansys2::Predicate("(light_on " + room_ + ")"));

    if (light_is_on && !is_occupied) {
      RCLCPP_INFO(get_logger(), "Room %s is unoccupied with light on - turning off", room_.c_str());
      
      // Remove the light_on predicate
      problem_expert_->removePredicate(
        plansys2::Predicate("(light_on " + room_ + ")"));
        
    } else if (light_is_on && is_occupied) {
      RCLCPP_INFO(get_logger(), "Room %s is occupied - leaving light on", room_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Room %s light already off", room_.c_str());
    }
  }

  float progress_;
  std::string room_;
  
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();

  node->set_parameter(rclcpp::Parameter("action_name", "patrol"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
