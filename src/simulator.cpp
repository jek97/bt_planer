#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bt_planner/action/move.hpp"

// ---------------------------------------------
//  Robot: holds the simulated pose
// ---------------------------------------------
class Robot
{
public:
  int x;
  int y;
  int theta;  // degrees, 0 = East, 90 = North, 180 = West, 270 = South

  Robot() : x(0), y(0), theta(0) {}

  void print_pose() const
  {
    RCLCPP_INFO(rclcpp::get_logger("Robot"),
      "Pose ? x: %d  y: %d  theta: %d�", x, y, theta);
  }
};

// ---------------------------------------------
//  Simulator node
// ---------------------------------------------
class Simulator : public rclcpp::Node
{
public:
  using Move        = bt_planner::action::Move;
  using GoalHandle  = rclcpp_action::ServerGoalHandle<Move>;

  Simulator() : Node("simulator")
  {
    action_server_ = rclcpp_action::create_server<Move>(
      this,
      "move",
      std::bind(&Simulator::handle_goal,   this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Simulator::handle_cancel, this, std::placeholders::_1),
      std::bind(&Simulator::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Simulator ready � action server 'move' is up.");
    robot_.print_pose();
  }

private:
  Robot robot_;
  rclcpp_action::Server<Move>::SharedPtr action_server_;

  // -- 1. Decide whether to accept the goal ----------------------------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const Move::Goal> goal)
  {
    const std::string & cmd = goal->command;

    if (cmd == "straight"    || cmd == "back"       ||
        cmd == "left"        || cmd == "right"       ||
        cmd == "turn_left"   || cmd == "turn_right")
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted: '%s'", cmd.c_str());
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    RCLCPP_WARN(this->get_logger(), "Unknown command '%s' � rejecting.", cmd.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // -- 2. Handle cancel requests ---------------------------------------------
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request received.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // -- 3. Execute the action in a detached thread ----------------------------
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&Simulator::execute, this, std::placeholders::_1),
                goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const std::string & cmd = goal_handle->get_goal()->command;

    // -- Dummy action: just print the command ------------------------------
    RCLCPP_INFO(this->get_logger(),
      "[execute] Received command: '%s'", cmd.c_str());

    // TODO: update robot_.x / y / theta based on cmd and grid logic

    // -- Return the (unchanged for now) robot pose -------------------------
    auto result  = std::make_shared<Move::Result>();
    result->x     = robot_.x;
    result->y     = robot_.y;
    result->theta = robot_.theta;

    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(),
      "[execute] Action succeeded. Returning pose ? x:%d y:%d theta:%d�",
      robot_.x, robot_.y, robot_.theta);
  }
};

// ---------------------------------------------
//  main
// ---------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
  return 0;
}