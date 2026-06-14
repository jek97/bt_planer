#include <chrono>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bt_planner/action/move.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using Move = bt_planner::action::Move;

// ---------------------------------------------
//  Environment map: grid[y][x] = 0 (free) | 1 (obstacle)
// ---------------------------------------------
struct EnvMap
{
  int size_x = 0;
  int size_y = 0;
  std::vector<std::vector<int>> grid;  // grid[y][x]
};

EnvMap parse_environment(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open())
    throw std::runtime_error("Cannot open environment file: " + path);

  json j;
  f >> j;

  EnvMap env;
  env.size_x = j.at("size").at("x").get<int>();
  env.size_y = j.at("size").at("y").get<int>();

  env.grid.assign(env.size_y, std::vector<int>(env.size_x, 0));

  for (const auto & obs : j.at("obstacles")) {
    int ox = obs.at("x").get<int>();
    int oy = obs.at("y").get<int>();
    if (ox >= 0 && ox < env.size_x && oy >= 0 && oy < env.size_y)
      env.grid[oy][ox] = 1;
  }

  return env;
}

// ---------------------------------------------
//  BT action node: MoveCommand
//  Non-blocking StatefulActionNode — reactive-safe.
// ---------------------------------------------
class MoveCommand : public BT::StatefulActionNode
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<Move>;

  MoveCommand(const std::string & name,
              const BT::NodeConfiguration & config,
              rclcpp::Node::SharedPtr node,
              rclcpp_action::Client<Move>::SharedPtr client)
  : BT::StatefulActionNode(name, config),
    node_(node),
    client_(client)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("command") };
  }

  // Called once when the node transitions from IDLE → RUNNING
  BT::NodeStatus onStart() override
  {
    auto cmd_opt = getInput<std::string>("command");
    if (!cmd_opt) {
      RCLCPP_ERROR(node_->get_logger(), "MoveCommand: missing 'command' port.");
      return BT::NodeStatus::FAILURE;
    }
    cmd_ = cmd_opt.value();

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "Action server not available.");
      return BT::NodeStatus::FAILURE;
    }

    Move::Goal goal;
    goal.command = cmd_;

    auto opts = rclcpp_action::Client<Move>::SendGoalOptions();
    goal_future_ = client_->async_send_goal(goal, opts);
    goal_handle_.reset();
    result_future_ = {};

    RCLCPP_INFO(node_->get_logger(), "[BT] Sending: '%s'", cmd_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  // Called every tick while RUNNING
  BT::NodeStatus onRunning() override
  {
    rclcpp::spin_some(node_);

    // Phase 1: wait for goal to be accepted
    if (!goal_handle_) {
      if (goal_future_.wait_for(std::chrono::milliseconds(0)) !=
          std::future_status::ready)
        return BT::NodeStatus::RUNNING;

      goal_handle_ = goal_future_.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "Goal '%s' rejected.", cmd_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      result_future_ = client_->async_get_result(goal_handle_);
      return BT::NodeStatus::RUNNING;
    }

    // Phase 2: wait for result
    if (result_future_.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
      return BT::NodeStatus::RUNNING;

    auto wrapped = result_future_.get();
    bool ok = wrapped.result->success;
    RCLCPP_INFO(node_->get_logger(),
      "[BT] '%s' -> x:%d y:%d theta:%d success:%s",
      cmd_.c_str(),
      wrapped.result->x, wrapped.result->y, wrapped.result->theta,
      ok ? "true" : "false");

    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // Called if a parent node halts this node mid-execution
  void onHalted() override
  {
    RCLCPP_WARN(node_->get_logger(), "[BT] '%s' halted, cancelling goal.", cmd_.c_str());
    if (goal_handle_)
      client_->async_cancel_goal(goal_handle_);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Move>::SharedPtr client_;
  std::string cmd_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
};

// ---------------------------------------------
//  Robot node
// ---------------------------------------------
class RobotNode : public rclcpp::Node
{
public:
  explicit RobotNode(const EnvMap & env)
  : Node("robot"), env_(env)
  {
    client_ = rclcpp_action::create_client<Move>(this, "move");

    RCLCPP_INFO(this->get_logger(),
      "Map loaded: %dx%d", env_.size_x, env_.size_y);
    log_map();
  }

  rclcpp_action::Client<Move>::SharedPtr client() { return client_; }

private:
  EnvMap env_;
  rclcpp_action::Client<Move>::SharedPtr client_;

  void log_map() const
  {
    for (int y = env_.size_y - 1; y >= 0; --y) {
      std::string row;
      for (int x = 0; x < env_.size_x; ++x)
        row += (env_.grid[y][x] ? "X" : ".");
      RCLCPP_INFO(this->get_logger(), "%s", row.c_str());
    }
  }
};

// ---------------------------------------------
//  main
// ---------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Resolve environment file path
  std::string env_path;
  if (argc >= 2) {
    env_path = argv[1];
  } else {
    env_path = ament_index_cpp::get_package_share_directory("bt_planner")
               + "/environment/env.json";
  }

  EnvMap env = parse_environment(env_path);

  auto robot_node = std::make_shared<RobotNode>(env);

  // --- Build the Behavior Tree ---
  BT::BehaviorTreeFactory factory;

  factory.registerBuilder<MoveCommand>(
    "MoveCommand",
    [robot_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<MoveCommand>(name, config,
                                           robot_node,
                                           robot_node->client());
    });

  std::string tree_path =
    ament_index_cpp::get_package_share_directory("bt_planner")
    + "/bt/main_tree.xml";

  auto tree = factory.createTreeFromFile(tree_path);

  RCLCPP_INFO(robot_node->get_logger(), "Behavior tree loaded. Ticking...");

  // --- Tick loop ---
  rclcpp::Rate rate(20);  // 20 Hz
  while (rclcpp::ok()) {
    rclcpp::spin_some(robot_node);

    BT::NodeStatus status = tree.tickRoot();

    if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(robot_node->get_logger(), "Tree returned FAILURE. Stopping.");
      break;
    }
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(robot_node->get_logger(), "Tree completed successfully.");
      break;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
