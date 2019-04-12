
#include "action_server.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

action_server::ActionServer::ActionServer(std::shared_ptr<rclcpp::Node> node)
{
  action_server_ = rclcpp_action::create_server<Fibonacci>(
    node,
    "fibonacci",
    std::bind(&action_server::ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&action_server::ActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&action_server::ActionServer::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse action_server::ActionServer::handle_goal(
  const std::array<uint8_t, 16> & uuid, std::shared_ptr<const Fibonacci::Goal> goal)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request with order %d", goal->order);
  (void)uuid;
  // Let's reject sequences that are over 9000
  if (goal->order > 9000)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::cout << "ACCEPT AND EXECUTE" << '\n';
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse action_server::ActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void action_server::ActionServer::execute(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  std::cout << "Entering ON EXECUTE" << '\n';
  RCLCPP_INFO(rclcpp::get_logger("server"), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  std::shared_ptr<example_interfaces::action::Fibonacci::Feedback> feedback = std::make_shared<Fibonacci::Feedback>();
  // printf(feedback);
  auto& sequence = feedback->sequence;
  sequence.push_back(0);
  sequence.push_back(1);
  auto result_response = std::make_shared<Fibonacci::Result>();

  for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
  {
    // Check if there is a cancel request
    if (goal_handle->is_canceling())
    {
      result_response->sequence = sequence;
      goal_handle->set_canceled(result_response);
      RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Canceled");
      return;
    }
    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Publish Feedback");

    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result_response->sequence = sequence;
    goal_handle->set_succeeded(result_response);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Suceeded");
  }
}

void action_server::ActionServer::handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread(&ActionServer::execute, this, goal_handle).detach();
}
