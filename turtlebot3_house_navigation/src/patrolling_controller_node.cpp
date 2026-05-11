#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include <plansys2_pddl_parser/Utils.h>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PatrollingController : public rclcpp::Node
{
public:
    enum State {
    WAITING_FOR_STATE,
    PATROL_CRITICAL,
    PATROL_HIGH,
    PATROL_LOW,
    EXECUTING,
    FINISHED,
    FAILED
  };

  PatrollingController()
  : Node("patrolling_controller"), state_(WAITING_FOR_STATE),should_exit_(false)
  {
    // Debug_mode to enable messages only for debgug.
    this->declare_parameter("debug_mode", 0);
    debug_mode_ = this->get_parameter("debug_mode").as_int();
  }
  bool shouldExit() const { return should_exit_; }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    // Subscribe to initial state from problem generator
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/initial_state", 10,
      std::bind(&PatrollingController::state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for initial state...");
  }

  void step()
  {
    switch (state_) {
      case WAITING_FOR_STATE:
        // Just wait for callback
        break;

      case PATROL_CRITICAL:
        execute_patrol(critical_room_, "CRITICAL", PATROL_HIGH);
        break;

      case PATROL_HIGH:
        execute_patrol(high_room_, "HIGH", PATROL_LOW);
        break;

      case PATROL_LOW:
        if (current_low_index_ < low_rooms_.size()) {
          execute_patrol(low_rooms_[current_low_index_], "LOW", PATROL_LOW);
        } else {
          RCLCPP_INFO(this->get_logger(), "\033[1;34m=== All rooms patrolled! ===\033[0m");
          state_ = FINISHED;
        }
        break;

      case EXECUTING:
        monitor_execution();
        break;

      case FINISHED:
        RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;36mPatrol complete. Thats all folks.\033[0m");
        should_exit_ = true;
        //rclcpp::shutdown();
        break;

      case FAILED:
        RCLCPP_ERROR_ONCE(this->get_logger(), "Patrol failed.");
        should_exit_ = true;
        //rclcpp::shutdown();
        break;
    }
  }

private:
  void state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
      if (state_ != WAITING_FOR_STATE) return;

      std::string data = msg->data;

      size_t crit_pos = data.find("critical:") + 9;
      size_t crit_end = data.find(";", crit_pos);
      critical_room_ = data.substr(crit_pos, crit_end - crit_pos);

      size_t high_pos = data.find("high:") + 5;
      size_t high_end = data.find(";", high_pos);
      high_room_ = data.substr(high_pos, high_end - high_pos);

      size_t low_pos = data.find("low:") + 4;
      std::string low_str = data.substr(low_pos);

      std::stringstream ss(low_str);
      std::string room;
      while (std::getline(ss, room, ',')) {
          if (!room.empty() && room.back() == ';')
              room.pop_back();
          low_rooms_.push_back(room);
      }

      std::string problem_str = problem_expert_->getProblem();
      size_t pos = problem_str.find("(robot_at ecobot ");
      std::string start_room_parsed;

      if (pos != std::string::npos) {
          size_t begin = pos + std::string("(robot_at ecobot ").size();
          size_t end = problem_str.find(")", begin);
          start_room_parsed = problem_str.substr(begin, end - begin);
      }

      low_rooms_.erase(
          std::remove(low_rooms_.begin(), low_rooms_.end(), start_room_parsed),
          low_rooms_.end()
      );

      low_rooms_.insert(low_rooms_.begin(), start_room_parsed);

      // There is a bug here but I can find it.
      RCLCPP_INFO(this->get_logger(),
          "Random Start room '%s' inserted first in patrol list",
          start_room_parsed.c_str());
      //-----------------------------------------------------------

      RCLCPP_INFO(this->get_logger(),
          "Received state - Critical: %s, High: %s, Low rooms: %zu",
          critical_room_.c_str(), high_room_.c_str(), low_rooms_.size());

      state_ = PATROL_CRITICAL;
  }


  void execute_patrol(const std::string & room, const std::string & priority, State next_state)
  {
    
    last_printed_pct_ = 0;
    
    RCLCPP_INFO(this->get_logger(), "\033[1;34mPatrolling %s room: %s\033[0m", priority.c_str(), room.c_str());

    std::string goal = "(and (patrolled " + room + "))";
    problem_expert_->setGoal(plansys2::Goal(goal));

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem(); 

    // Debug: print current problem state
    if (debug_mode_) {
      RCLCPP_INFO(this->get_logger(), "Current problem:\n%s", problem.c_str());
    }  

    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR(this->get_logger(), "No plan found for %s", room.c_str());
      state_ = FAILED;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Plan has %zu actions", plan.value().items.size());

    if (executor_client_->start_plan_execution(plan.value())) {
      next_state_after_execution_ = next_state;
      current_room_ = room;
      state_ = EXECUTING;
    } else {
      state_ = FAILED;
    }
  }


  void monitor_execution()
  {
    auto feedback = executor_client_->getFeedBack();

    // A bit of a hassle to write all this, but otherwise there are too many messages even duirng debug.
    for (const auto & action_feedback : feedback.action_execution_status) {
      if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
        if (debug_mode_) {
          int pct = static_cast<int>(action_feedback.completion * 100.0);
          if (pct >= 25 && last_printed_pct_ < 25) {
            RCLCPP_INFO(this->get_logger(), "[%s 25%%]", action_feedback.action.c_str());
            last_printed_pct_ = 25;
          } else if (pct >= 50 && last_printed_pct_ < 50) {
            RCLCPP_INFO(this->get_logger(), "[%s 50%%]", action_feedback.action.c_str());
            last_printed_pct_ = 50;
          } else if (pct >= 75 && last_printed_pct_ < 75) {
            RCLCPP_INFO(this->get_logger(), "[%s 75%%]", action_feedback.action.c_str());
            last_printed_pct_ = 75;
          } else if (pct >= 100 && last_printed_pct_ < 100) {
            RCLCPP_INFO(this->get_logger(), "[%s 100%%]", action_feedback.action.c_str());
            last_printed_pct_ = 100;
          }
        }
      }
    }

    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(this->get_logger(), "\033[1;34mCompleted patrol of %s\033[0m", current_room_.c_str());

        // Clean up patrolled predicate for next iteration
        problem_expert_->removePredicate(
          plansys2::Predicate("(patrolled " + current_room_ + ")"));

        // Move to next state
        if (next_state_after_execution_ == PATROL_LOW) {
          current_low_index_++;
        }
        state_ = next_state_after_execution_;

      } else {
        RCLCPP_ERROR(this->get_logger(), "Execution failed for %s", current_room_.c_str());
        last_printed_pct_ = 0;  // reset for replan
        state_ = FAILED;
      }
    }
  }

  State state_;
  State next_state_after_execution_;

  std::string critical_room_;
  std::string high_room_;
  std::vector<std::string> low_rooms_;
  size_t current_low_index_ = 0;
  std::string current_room_;
  bool should_exit_ = false;
  int debug_mode_;
  int last_printed_pct_ = 0;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();
  node->init();
  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    node->step();
    
    // Check if we should exit
    if (!rclcpp::ok()) {
      break;
    }
    
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }
  rclcpp::shutdown();
  return 0;
}

// Text Color - make easy to read the logs.
// Color  Code             Bright/Bold
// Red    \033[0;31m       \033[1;31m
// Orange \033[0;38;5;208m \033[1;38;5;208m
// Green  \033[0;32m       \033[1;32m
// Yellow \033[0;33m       \033[1;33m
// Blue   \033[0;34m       \033[1;34m
// Cyan   \033[0;36m       \033[1;36m
// Reset  \033[0m         (Always at end)
