#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TurnOffLightAction : public plansys2::ActionExecutorClient
{
public:
  TurnOffLightAction()
  : plansys2::ActionExecutorClient("turn_off_light", 250ms)
  {
    // Publisher to notify other nodes about light changes
    light_pub_ = this->create_publisher<std_msgs::msg::String>("/light_turned_off", 10);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    progress_ = 0.0;
    
    // Get room name from PDDL action arguments: (turn_off_light ecobot <room>)
    room_ = get_arguments()[1];
    
    RCLCPP_INFO(get_logger(), "Turning off light in %s", room_.c_str());
    send_feedback(0.0, "Turning off light");

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.25;  // 4 ticks × 250ms = 1 second total
      send_feedback(progress_, "Turning off light in " + room_);
    } else {
      // Publish notification
      auto msg = std_msgs::msg::String();
      msg.data = room_;
      light_pub_->publish(msg);

      RCLCPP_INFO(get_logger(), "Light turned off in %s", room_.c_str());
      finish(true, 1.0, "Light turned off");
    }
  }

  float progress_;
  std::string room_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr light_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurnOffLightAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "turn_off_light"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
