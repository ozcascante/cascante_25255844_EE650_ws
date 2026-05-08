// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller"), state_(STARTING)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();


    // Wait for PlanSys2 lifecycle nodes to become active
    rclcpp::sleep_for(std::chrono::seconds(5));

    // Load the waypoint list from the rooms.yaml
    this->declare_parameter("waypoints", std::vector<std::string>());
    waypoints_ = this->get_parameter("waypoints").as_string_array();

    // DEBUG PRINT:
    //RCLCPP_INFO(this->get_logger(), "Waypoints loaded: %zu", waypoints_.size());
    //for (const auto & w : waypoints_) {
    //    RCLCPP_INFO(this->get_logger(), " - Found room: '%s'", w.c_str());
    //}

    init_knowledge();
  }

  void init_knowledge()
  {

    problem_expert_->addInstance(plansys2::Instance{"ecobot", "robot"});
  

    // Add all rooms from the YAML list
    for (const auto & room : waypoints_) {
      if (!problem_expert_->addInstance(plansys2::Instance{room, "room"})) {
         // If it fails here, it's usually because the room is already there
        RCLCPP_DEBUG(this->get_logger(), "Room %s already exists.", room.c_str());
      }
    }

    this->declare_parameter("start_room_name", "utility");
    std::string start_room = this->get_parameter("start_room_name").as_string();
    problem_expert_->addPredicate(
        plansys2::Predicate("(robot_at ecobot " + start_room + ")"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected dining kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen dining)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen utility)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected utility kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected dining hallway)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected hallway dining)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected hallway bedroom1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom1 hallway)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom1 bedroom2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom2 bedroom1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom1 bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bathroom bedroom1)"));

  }

  void step()
  {
    switch (state_) {
      case STARTING:
        {
          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and (patrolled kitchen))"));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = PATROL_KITCHEN;
          }
        }
        break;
      case PATROL_KITCHEN:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled kitchen)"));

              // Set the goal for next state
              problem_expert_->setGoal(plansys2::Goal("(and (patrolled utility))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = PATROL_UTILITY;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      case PATROL_UTILITY:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled utility)"));

              // Set the goal for next state
              problem_expert_->setGoal(plansys2::Goal("(and (patrolled hallway))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = PATROL_HALLWAY;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      case PATROL_HALLWAY:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled hallway)"));

              // Set the goal for next state
              problem_expert_->setGoal(plansys2::Goal("(and (patrolled bedroom1))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = PATROL_BEDROOM1;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      case PATROL_BEDROOM1:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled bedroom1)"));

              // Set the goal for next state
              problem_expert_->setGoal(plansys2::Goal("(and (patrolled bedroom2))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                // Loop to WP1
                state_ = PATROL_BEDROOM2;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      case PATROL_BEDROOM2:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled bedroom2)"));

              // Set the goal for next state
              problem_expert_->setGoal(plansys2::Goal("(and (patrolled kitchen))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                // Loop to WP1
                state_ = PATROL_KITCHEN;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }

        break;
      default:
        break;
    }
  }

private:
  typedef enum {STARTING, PATROL_KITCHEN, PATROL_UTILITY, PATROL_HALLWAY, PATROL_BEDROOM1, PATROL_BEDROOM2} StateType;
  StateType state_;

  //Start variable to track rooms
  std::vector<std::string> waypoints_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
