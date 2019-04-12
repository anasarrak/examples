#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
namespace action_server
{
  class ActionServer
  {
    public:
      ActionServer(std::shared_ptr<rclcpp::Node> node);

    private:
      void create_action(std::shared_ptr<rclcpp::Node> node);
      rclcpp_action::GoalResponse handle_goal(
        const std::array<uint8_t, 16> & uuid, std::shared_ptr<const Fibonacci::Goal> goal);

      rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle);

      void execute(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle);

      void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle);

      std::shared_ptr<rclcpp_action::Server<Fibonacci>> action_server_;

      // std::unique_ptr<ActionServer> impl_;
  };
}
