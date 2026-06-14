#include <memory>
#include <string>
#include <thread>
#include <fstream>
#include <set>
#include <mutex>
#include <cmath>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bt_planner/action/move.hpp"

#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <SFML/Graphics.hpp>

using json = nlohmann::json;

static const int WINDOW_W = 1000;
static const int WINDOW_H = 1000;

// ---------------------------------------------
//  Environment
// ---------------------------------------------
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
    env.obstacles.insert({obs.at("x").get<int>(), obs.at("y").get<int>()});
  }
  return env;
}

// ---------------------------------------------
//  Robot
// ---------------------------------------------
class Robot
{
public:
  int x     = 0;
  int y     = 0;
  int theta = 0;  // 0=up, 90=right, 180=down, 270=left

  void print_pose() const
  {
    RCLCPP_INFO(rclcpp::get_logger("Robot"),
      "Pose: x=%d  y=%d  theta=%d", x, y, theta);
  }
};

// ---------------------------------------------
//  Rendering helpers
// ---------------------------------------------
static void draw_arrow(sf::RenderTarget & target,
                       sf::Vector2f center, int theta, float size)
{
  // Convert theta to radians: 0=up means -π/2 in standard screen angle
  float angle_rad;
  switch (theta) {
    case 0:   angle_rad = -M_PI / 2.f; break;  // up
    case 90:  angle_rad = 0.f;          break;  // right
    case 180: angle_rad = M_PI / 2.f;  break;  // down
    default:  angle_rad = M_PI;         break;  // left (270)
  }

  sf::Vector2f dir(std::cos(angle_rad), std::sin(angle_rad));
  sf::Vector2f perp(-dir.y, dir.x);

  sf::Vector2f tip   = center + dir * size;
  sf::Vector2f base  = center - dir * (size * 0.4f);
  sf::Vector2f left  = base + perp * (size * 0.35f);
  sf::Vector2f right = base - perp * (size * 0.35f);

  sf::ConvexShape arrow(3);
  arrow.setPoint(0, tip);
  arrow.setPoint(1, left);
  arrow.setPoint(2, right);
  arrow.setFillColor(sf::Color::White);
  target.draw(arrow);
}

void render_environment(sf::RenderTarget & target,
                        const Environment & env,
                        const Robot & robot)
{
  if (env.size_x <= 0 || env.size_y <= 0) return;

  float cell_w = static_cast<float>(WINDOW_W) / env.size_x;
  float cell_h = static_cast<float>(WINDOW_H) / env.size_y;

  target.clear(sf::Color::White);

  for (int gy = 0; gy < env.size_y; ++gy) {
    for (int gx = 0; gx < env.size_x; ++gx) {
      float px = gx * cell_w;
      // Flip y: row 0 of the grid is at the bottom of the window
      float py = (env.size_y - 1 - gy) * cell_h;

      sf::RectangleShape cell(sf::Vector2f(cell_w - 1.f, cell_h - 1.f));
      cell.setPosition(px, py);

      bool is_obstacle = env.obstacles.count({gx, gy}) > 0;
      bool is_robot    = (gx == robot.x && gy == robot.y);

      if (is_obstacle) {
        cell.setFillColor(sf::Color::Black);
      } else if (is_robot) {
        cell.setFillColor(sf::Color::Red);
      } else {
        cell.setFillColor(sf::Color::White);
      }
      cell.setOutlineColor(sf::Color(180, 180, 180));
      cell.setOutlineThickness(1.f);
      target.draw(cell);

      if (is_robot) {
        sf::Vector2f center(px + cell_w / 2.f, py + cell_h / 2.f);
        draw_arrow(target, center, robot.theta,
                   std::min(cell_w, cell_h) * 0.35f);
      }
    }
  }
}

// ---------------------------------------------
//  Simulator node
// ---------------------------------------------
class Simulator : public rclcpp::Node
{
public:
  using Move       = bt_planner::action::Move;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Move>;

  explicit Simulator(const std::string & env_path)
  : Node("simulator")
  {
    env_ = parse_environment(env_path);
    RCLCPP_INFO(this->get_logger(),
      "Environment loaded: %dx%d, %zu obstacle(s).",
      env_.size_x, env_.size_y, env_.obstacles.size());

    action_server_ = rclcpp_action::create_server<Move>(
      this, "move",
      std::bind(&Simulator::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Simulator::handle_cancel,   this, std::placeholders::_1),
      std::bind(&Simulator::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Simulator ready. Action server 'move' is up.");
    robot_.print_pose();
  }

  // Called from the main render loop — thread-safe snapshot
  void render(sf::RenderTarget & target)
  {
    std::lock_guard<std::mutex> lock(robot_mutex_);
    render_environment(target, env_, robot_);
  }

private:
  Robot       robot_;
  Environment env_;
  std::mutex  robot_mutex_;
  rclcpp_action::Server<Move>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
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

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&Simulator::execute, this, std::placeholders::_1),
                goal_handle}.detach();
  }

  // Direction vector for a given theta (grid coords: y increases upward)
  static std::pair<int,int> direction(int theta)
  {
    switch (theta) {
      case 0:   return { 0,  1};  // up
      case 90:  return { 1,  0};  // right
      case 180: return { 0, -1};  // down
      default:  return {-1,  0};  // left (270)
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const std::string & cmd = goal_handle->get_goal()->command;
    RCLCPP_INFO(this->get_logger(), "[execute] command: '%s'", cmd.c_str());

    {
      std::lock_guard<std::mutex> lock(robot_mutex_);

      if (cmd == "turn_left") {
        robot_.theta = (robot_.theta - 90 + 360) % 360;
      } else if (cmd == "turn_right") {
        robot_.theta = (robot_.theta + 90) % 360;
      } else {
        auto [dx, dy] = direction(robot_.theta);

        if      (cmd == "back")  { dx = -dx; dy = -dy; }
        else if (cmd == "left")  { std::swap(dx, dy); dx = -dx; }
        else if (cmd == "right") { std::swap(dx, dy); dy = -dy; }
        // "straight": dx/dy unchanged

        int nx = robot_.x + dx;
        int ny = robot_.y + dy;

        bool in_bounds  = nx >= 0 && nx < env_.size_x &&
                          ny >= 0 && ny < env_.size_y;
        bool free_cell  = env_.obstacles.count({nx, ny}) == 0;

        if (in_bounds && free_cell) {
          robot_.x = nx;
          robot_.y = ny;
        } else {
          RCLCPP_WARN(this->get_logger(),
            "Move blocked at (%d,%d).", nx, ny);
        }
      }
    }

    robot_.print_pose();

    auto result   = std::make_shared<Move::Result>();
    {
      std::lock_guard<std::mutex> lock(robot_mutex_);
      result->x     = robot_.x;
      result->y     = robot_.y;
      result->theta = robot_.theta;
    }
    goal_handle->succeed(result);
  }
};

// ---------------------------------------------
//  main – SFML render loop + ROS2 spin_some
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

  sf::RenderWindow window(
    sf::VideoMode(WINDOW_W, WINDOW_H), "Robot Environment",
    sf::Style::Titlebar | sf::Style::Close);
  window.setFramerateLimit(30);

  while (window.isOpen() && rclcpp::ok()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    rclcpp::spin_some(node);

    node->render(window);
    window.display();
  }

  rclcpp::shutdown();
  return 0;
}
