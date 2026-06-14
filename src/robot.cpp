#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bt_planner/action/move.hpp"

class RobotNode : public rclcpp::Node
{
public:
  using Move       = bt_planner::action::Move;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Move>;

  RobotNode() : Node("robot")
  {
    client_ = rclcpp_action::create_client<Move>(this, "move");
  }

  // Send a single command synchronously; returns the success flag.
  bool send_command(const std::string & cmd)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available.");
      return false;
    }

    auto goal = Move::Goal();
    goal.command = cmd;

    auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();

    auto future_goal = client_->async_send_goal(goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future_goal) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal '%s'.", cmd.c_str());
      return false;
    }

    auto goal_handle = future_goal.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal '%s' rejected.", cmd.c_str());
      return false;
    }

    auto future_result = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result for '%s'.", cmd.c_str());
      return false;
    }

    auto result = future_result.get();
    RCLCPP_INFO(this->get_logger(),
      "[%s] -> x:%d y:%d theta:%d success:%s",
      cmd.c_str(),
      result.result->x,
      result.result->y,
      result.result->theta,
      result.result->success ? "true" : "false");

    return result.result->success;
  }

  void run()
  {
    const std::vector<std::string> sequence = {
      "straight", "turn_right", "straight", "turn_left", "straight"
    };

    while (rclcpp::ok()) {
      for (const auto & cmd : sequence) {
        if (!send_command(cmd)) {
          RCLCPP_WARN(this->get_logger(),
            "Command '%s' failed. Stopping.", cmd.c_str());
          return;
        }
      }
      RCLCPP_INFO(this->get_logger(), "Sequence complete, repeating...");
    }
  }

private:
  rclcpp_action::Client<Move>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
