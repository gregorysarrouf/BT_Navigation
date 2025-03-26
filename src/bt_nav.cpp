#include <iostream>
#include <chrono>
#include <vector>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

int battery_level = 100;
int locIdx = 0;
const std::vector<std::vector<float>> locationCoords = {
  { 0.0f, -2.0f },    // Point A (Right)
  { 2.0f, 0.0f },     // Point B (Up)
  { 0.0f, 2.0f },     // Point C (Left)
  { -2.0f, 0.0f },    // Point D (Down)
  { -0.55f, 0.55f }   // Charging Station   
};
const std::vector<std::string> locationNames = {
  "Point A",
  "Point B",
  "Point C",
  "Point D",
  "Charging Station"
};

BT::NodeStatus isBatteryLow() {
  if (battery_level < 25){
   return  BT::NodeStatus::SUCCESS;
  }
  else{
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus setChargingLocation() {
  locIdx = 4;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus fillBattery() {
  battery_level = 100;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus decreaseBattery() {
  battery_level -= 20;
  std::cout << "Battery is now " << battery_level << "%" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class GoToLocation : public BT::SyncActionNode {
  public:
      GoToLocation(const std::string& name) : BT::SyncActionNode(name, {}) {}
  
      BT::NodeStatus tick() override {
          std::string moveCommand = 
            "../scripts/nav_to_location.py " + 
            std::to_string(locationCoords[locIdx][0]) +
	    " " +
            std::to_string(locationCoords[locIdx][1]);

          std::cout << "Navigating to: " << locationNames[locIdx] << std::endl;
	  system(moveCommand.c_str());

	  if (locIdx != 4) {
	      std::string spinCommand = "../scripts/spin_robot.py";
	      std::cout << "Cleaning Location: " << locationNames[locIdx] << std::endl;
	      system(spinCommand.c_str());
	  }

          locIdx = (locIdx == 4) ? 0 : locIdx = (locIdx + 1) % 4;

	  return BT::NodeStatus::SUCCESS;
      }
  };
  
  class Recharge : public BT::SyncActionNode {
  public:
      Recharge(const std::string& name) : BT::SyncActionNode(name, {}) {}
  
      BT::NodeStatus tick() override {
          std::cout << "Recharging for 30s..." << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(30));
          return BT::NodeStatus::SUCCESS;
      }
  };
  


int main()
{
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<GoToLocation>("goToLocation");
  factory.registerNodeType<Recharge>("recharge");

  factory.registerSimpleCondition("isBatteryLow", std::bind(isBatteryLow));
  factory.registerSimpleAction("setChargingLocation", std::bind(setChargingLocation));
  factory.registerSimpleAction("fillBattery", std::bind(fillBattery));
  factory.registerSimpleAction("decreaseBattery", std::bind(decreaseBattery));

  // Create Tree
  auto tree = factory.createTreeFromFile("./../bt_tree.xml");

  // Execute the tree
  while (true){
    tree.tickWhileRunning();
    std::cout << "Iteration complete\n\n";
  }

  return 0;
}
