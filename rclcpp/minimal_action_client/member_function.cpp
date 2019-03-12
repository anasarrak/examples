// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

class MinimalActionClient : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  MinimalActionClient(const std::string node_name)
    : Node(node_name)
  {
  }

  void spin_until_goal_complete(const int goal_order)
  {
    using namespace std::placeholders;

    auto client_ptr =
      rclcpp_action::create_client<Fibonacci>(this->shared_from_this(), "fibonacci");

    if (!client_ptr->wait_for_action_server(std::chrono::seconds(20))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = goal_order;

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto goal_handle_future = client_ptr->async_send_goal(
      goal_msg,
      std::bind(&MinimalActionClient::feedback_callback, this, _1, _2));

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), goal_handle_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Send goal call failed :(");
      return;
    }

    GoalHandleFibonacci::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return;
    }

    // Wait for the server to be done with the goal
    auto result_future = goal_handle->async_result();

    RCLCPP_INFO(this->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Get result call failed :(");
      return;
    }

    GoalHandleFibonacci::WrappedResult wrapped_result = result_future.get();

    switch(wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : wrapped_result.result->sequence)
    {
      RCLCPP_INFO(this->get_logger(), "%" PRId64, number);
    }
  }

private:
  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Next number in sequence received: %" PRId64,
      feedback->sequence.back());
  }
};  // class MinimalActionClient

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MinimalActionClient>("minimal_action_client");

  action_client->spin_until_goal_complete(10);

  rclcpp::shutdown();
  return 0;
}
