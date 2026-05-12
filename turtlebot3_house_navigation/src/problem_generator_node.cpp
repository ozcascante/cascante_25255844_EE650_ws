#include <memory>
#include <string>
#include <vector>
#include <map>
#include <random>
#include <algorithm>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

using namespace std::chrono_literals;

class ProblemGeneratorNode : public rclcpp::Node
{
public:
  ProblemGeneratorNode()
  : Node("problem_generator_node")
  {
    this->declare_parameter("waypoints", std::vector<std::string>());
    waypoints_ = this->get_parameter("waypoints").as_string_array();

    this->declare_parameter("start_room_name", "");
    start_room_ = this->get_parameter("start_room_name").as_string();

    // debug_mode - avoid display so many messages. Only for debug.
    this->declare_parameter("debug_mode", 0);
    debug_mode_ = this->get_parameter("debug_mode").as_int();

    // Publisher for initial state (controller subscribes)
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/initial_state", 10);
  }

  void init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();

    RCLCPP_INFO(this->get_logger(), "Waiting for PlanSys2...");
    rclcpp::sleep_for(5s);

    // Create Room Priority, Room occupancy and Light states
    generate_random_state();
    // Build the problem for PlanSys2
    populate_problem();
    // Publish the problem to topic /initial_state
    publish_state();

    RCLCPP_INFO(this->get_logger(), "Problem generation complete");
  }

private:
  void generate_random_state()
  {
    std::random_device rd;
    std::mt19937 gen(rd());

    // Shuffle rooms for energy assignment
    std::vector<std::string> shuffled = waypoints_;
    std::shuffle(shuffled.begin(), shuffled.end(), gen);

    critical_room_ = shuffled[0];
    high_room_ = shuffled[1];
    for (size_t i = 2; i < shuffled.size(); ++i) {
      low_rooms_.push_back(shuffled[i]);
    }

    // Random occupancy and lights
    std::bernoulli_distribution coin(0.5);
    for (const auto & room : waypoints_) {
      occupied_[room] = coin(gen);
      light_on_[room] = coin(gen);
    }

    // Read start room from file
    std::string start_room_file = "/tmp/patrol_start_room.txt";
    std::ifstream infile(start_room_file);
    if (infile.good()) {
      std::getline(infile, start_room_);
      RCLCPP_INFO(this->get_logger(),
        "Read start room from file: %s", start_room_.c_str());
    } else {
      std::uniform_int_distribution<size_t> dist(0, waypoints_.size() - 1);
      start_room_ = waypoints_[dist(gen)];
      RCLCPP_WARN(this->get_logger(),
        "Could not read start room file, using random: %s",
        start_room_.c_str());
    }
    infile.close();

    // Log state
    RCLCPP_INFO(this->get_logger(), "====================================");
    RCLCPP_INFO(this->get_logger(), "====== Generated Initial State =====");
    RCLCPP_INFO(this->get_logger(), "====================================");
    RCLCPP_INFO(this->get_logger(),
      "\033[1;36mRandom Start Room: %s\033[0m", start_room_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "\033[1;31mCritical: %s\033[0m", critical_room_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "\033[1;38;5;208mHigh: %s\033[0m", high_room_.c_str());

    std::string low_str;
    for (const auto & r : low_rooms_) low_str += r + " ";
    RCLCPP_INFO(this->get_logger(),
      "\033[1;32mLow: %s\033[0m", low_str.c_str());

    for (const auto & room : waypoints_) {
      RCLCPP_INFO(this->get_logger(),
        "\033[1;34m  %s: occupied=%s, light=%s\033[0m",
        room.c_str(),
        occupied_[room] ? "yes" : "no",
        light_on_[room] ? "on" : "off");
    }
  }

  void populate_problem()
  {
    problem_expert_->clearKnowledge();

    // Addin Robot to problem: ecobot sounded like a nice name for our project
    problem_expert_->addInstance(plansys2::Instance{"ecobot", "robot"});

    // Energy level constants must be objects for PlanSys2
    problem_expert_->addInstance(plansys2::Instance{"low", "energy"});
    problem_expert_->addInstance(plansys2::Instance{"high", "energy"});
    problem_expert_->addInstance(plansys2::Instance{"critical", "energy"});

    // Adding the rooms rooms read from config file
    for (const auto & room : waypoints_) {
      problem_expert_->addInstance(plansys2::Instance{room, "room"});
    }

    // Robot Starting Position from temp file written during launch file run to start gazebo.
    problem_expert_->addPredicate(
      plansys2::Predicate("(robot_at ecobot " + start_room_ + ")"));

    // Occupancy from random coin toast at the start.
    for (const auto & [room, occ] : occupied_) {
      if (occ) {
        problem_expert_->addPredicate(
          plansys2::Predicate("(occupied " + room + ")"));
      }
    }

    // Lights from random coin toast at the start.
    for (const auto & [room, on] : light_on_) {
      if (on) {
        problem_expert_->addPredicate(
          plansys2::Predicate("(light_on " + room + ")"));
      }
    }

    // Energy level predicates: critical, high, low. This will be useful if we round the patrols but our case
    // with a single patrol is an excercise only.   

    // Critical Energy level. Only one room should have it. And by the first robot round we end with high (lights on) and low (lights off)
    problem_expert_->addPredicate(
      plansys2::Predicate("(energy_level " + critical_room_ + " critical)"));

    // High Level (lights on)
    problem_expert_->addPredicate(
      plansys2::Predicate("(energy_level " + high_room_ + " high)"));

    // Low (lights off)
    for (const auto & room : low_rooms_) {
      problem_expert_->addPredicate(
        plansys2::Predicate("(energy_level " + room + " low)"));
    }

    // Room Connections. They are hardode in the program. We should move to config file so is easy to remove rooms or add rooms.
    std::vector<std::pair<std::string, std::string>> connections = {
      {"dining", "kitchen"}, {"kitchen", "dining"},
      {"kitchen", "utility"}, {"utility", "kitchen"},
      {"dining", "bedroom1"}, {"bedroom1", "dining"},
    //  {"dining", "hallway"}, {"hallway", "dining"}, // Took it off. Not really helping if patrol in the hallway.
    //  {"hallway", "bedroom1"}, {"bedroom1", "hallway"},// Robot gets lost after spinning.
      {"bedroom1", "bedroom2"}, {"bedroom2", "bedroom1"},
      {"bedroom1", "bathroom"}, {"bathroom", "bedroom1"}
    };

    for (const auto & [r1, r2] : connections) {
      problem_expert_->addPredicate(
        plansys2::Predicate("(connected " + r1 + " " + r2 + ")"));
    }

    RCLCPP_INFO(this->get_logger(), "Problem populated in PlanSys2");
  }

  void publish_state()
  {
    // Format: critical:room;high:room;low:room1,room2,...;
    std::string state_str = "critical:" + critical_room_
                          + ";high:" + high_room_
                          + ";low:";

    for (size_t i = 0; i < low_rooms_.size(); ++i) {
      state_str += low_rooms_[i];
      if (i < low_rooms_.size() - 1) state_str += ",";
    }

    state_str += ";";   // Required to avoid parsing bug and missing a room

    auto msg = std_msgs::msg::String();
    msg.data = state_str;

    for (int i = 0; i < 10; ++i) {
      state_pub_->publish(msg);
      rclcpp::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(),
      "Published state: %s", state_str.c_str());
  }

  std::vector<std::string> waypoints_;
  std::string start_room_;
  int debug_mode_;

  std::string critical_room_;
  std::string high_room_;
  std::vector<std::string> low_rooms_;
  std::map<std::string, bool> occupied_;
  std::map<std::string, bool> light_on_;

  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProblemGeneratorNode>();
  node->init();
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
