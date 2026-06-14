#include <memory>
#include <string>
#include <thread>
#include <fstream>
#include <set>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bt_planner/action/move.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using json = nlohmann::json;

// ---------------------------------------------
//  Environment: grid size and obstacle set
// ---------------------------------------------
struct Cell { int x; int y; };

struct Environment
{
  int size_x = 0;
  int size_y = 0;
  std::set<std::pair<int,int>> obstacles;
};

Environment parse_environment(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open())
    throw std::runtime_error("Cannot open environment file: " + path);

  json j;
  f >> j;

  Environment env;
  env.size_x = j.at("size").at("x").get<int>();
  env.size_y = j.at("size").at("y").get<int>();

  for (const auto & obs : j.at("obstacles")) {
    int ox = obs.at("x").get<int>();
    int oy = obs.at("y").get<int>();
    env.obstacles.insert({ox, oy});
  }

  return env;
}

// ---------------------------------------------
//  Robot: holds the simulated pose
// ---------------------------------------------
class Robot
{
public:
  int x;
  int y;
  int theta;  // degrees: 0=up, 90=right, 180=down, 270=left

  Robot() : x(0), y(0), theta(0) {}

  void print_pose() const
  {
    RCLCPP_INFO(rclcpp::get_logger("Robot"),
      "Pose: x=%d  y=%d  theta=%d deg", x, y, theta);
  }
};

// ---------------------------------------------
//  Visualization
// ---------------------------------------------
static const int WINDOW_W = 1000;
static const int WINDOW_H = 1000;

void draw_environment(const Environment & env, const Robot & robot)
{
  if (env.size_x <= 0 || env.size_y <= 0) return;

  int cell_w = WINDOW_W / env.size_x;
  int cell_h = WINDOW_H / env.size_y;

  cv::Mat img(WINDOW_H, WINDOW_W, CV_8UC3, cv::Scalar(255, 255, 255));

  // Draw cells
  for (int gy = 0; gy < env.size_y; ++gy) {
    for (int gx = 0; gx < env.size_x; ++gx) {
      int px = gx * cell_w;
      // Flip y so row 0 is at bottom
      int py = (env.size_y - 1 - gy) * cell_h;
      cv::Rect cell_rect(px, py, cell_w, cell_h);

      if (env.obstacles.count({gx, gy})) {
        cv::rectangle(img, cell_rect, cv::Scalar(0, 0, 0), cv::FILLED);
      } else if (gx == robot.x && gy == robot.y) {
        cv::rectangle(img, cell_rect, cv::Scalar(0, 0, 220), cv::FILLED);

        // Arrow direction based on theta
        // Center of cell in image coordinates
        cv::Point center(px + cell_w / 2, py + cell_h / 2);
        int arrow_len = std::min(cell_w, cell_h) * 2 / 5;

        cv::Point tip;
        if (robot.theta == 0)         tip = center + cv::Point(0, -arrow_len);   // up
        else if (robot.theta == 90)   tip = center + cv::Point(arrow_len, 0);    // right
        else if (robot.theta == 180)  tip = center + cv::Point(0, arrow_len);    // down
        else                          tip = center + cv::Point(-arrow_len, 0);   // left (270)

        cv::arrowedLine(img, center, tip, cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0, 0.4);
      }

      // Grid lines
      cv::rectangle(img, cell_rect, cv::Scalar(180, 180, 180), 1);
    }
  }

  cv::imshow("Environment", img);
  cv::waitKey(1);
}

// ---------------------------------------------
//  Simulator node
// ---------------------------------------------
class Simulator : public rclcpp::Node
{
public:
  using Move        = bt_planner::action::Move;
  using GoalHandle  = rclcpp_action::ServerGoalHandle<Move>;

  explicit Simulator(const std::string & env_path)
  : Node("simulator")
  {
    env_ = parse_environment(env_path);
    RCLCPP_INFO(this->get_logger(),
      "Environment loaded: %dx%d, %zu obstacle(s).",
      env_.size_x, env_.size_y, env_.obstacles.size());

    draw_environment(env_, robot_);

    action_server_ = rclcpp_action::create_server<Move>(
      this,
      "move",
      std::bind(&Simulator::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Simulator::handle_cancel,   this, std::placeholders::_1),
      std::bind(&Simulator::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Simulator ready. Action server 'move' is up.");
    robot_.print_pose();
  }

private:
  Robot       robot_;
  Environment env_;
  rclcpp_action::Server<Move>::SharedPtr action_server_;

  // -- 1. Decide whether to accept the goal ----------------------------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const Move::Goal> goal)
  {
    const std::string & cmd = goal->command;

    if (cmd == "straight"  || cmd == "back"       ||
        cmd == "left"      || cmd == "right"       ||
        cmd == "turn_left" || cmd == "turn_right")
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted: '%s'", cmd.c_str());
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    RCLCPP_WARN(this->get_logger(), "Unknown command '%s' – rejecting.", cmd.c_str());
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

    RCLCPP_INFO(this->get_logger(), "[execute] command: '%s'", cmd.c_str());

    // TODO: update robot_.x / y / theta based on cmd and grid logic
    //       then call draw_environment(env_, robot_) to refresh the view

    auto result    = std::make_shared<Move::Result>();
    result->x      = robot_.x;
    result->y      = robot_.y;
    result->theta  = robot_.theta;

    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(),
      "[execute] succeeded. Pose: x=%d y=%d theta=%d",
      robot_.x, robot_.y, robot_.theta);
  }
};

// ---------------------------------------------
//  main
// ---------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string env_path;
  if (argc >= 2) {
    env_path = argv[1];
  } else {
    env_path = ament_index_cpp::get_package_share_directory("bt_planner")
               + "/environment/env.json";
  }

  auto node = std::make_shared<Simulator>(env_path);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
