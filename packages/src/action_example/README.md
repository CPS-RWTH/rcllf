# action_example

Demonstrates the ROS2 action pattern: a server that executes long-running goals with feedback, and a timer-driven client.

## Pattern

**Action server node:** `ActionServerNode` (extends `rclcpp::Node`)

| Element | Type | Detail |
|---------|------|--------|
| Action server | `rclcpp_action::create_server` | Name: `analyze`, Type: `custom_interfaces::action::Analyze` |
| Callback | `handle_goal` | Parameters: `uuid`, `goal` |
| Callback | `handle_cancel` | Parameters: `goal_handle` |
| Callback | `handle_accepted` | Parameters: `goal_handle` |
| Timer | `create_wall_timer` | Period: 100ms, `step_goals` advances every active goal by one sample (feedback, then the result) |
| Shutdown | pre-shutdown callback, destructor | Aborts every active goal before the context goes down. Under lfc the callback runs in the node's `reaction(shutdown)` |

**Action client node:** `ActionClientNode` (extends `rclcpp::Node`)

| Element | Type | Detail |
|---------|------|--------|
| Timer | `create_wall_timer` | Period: 2s |
| Callback | `timer_callback` | Parameters: *(none)* |
| Action client | `rclcpp_action::create_client` | Action: `analyze` |
| Callback | `feedback_callback` | Parameters: `goal_handle`, `feedback` |
| Callback | `result_callback` | Parameters: `result` |

## Callback Signatures

```cpp
// Goal handler — decides whether to accept or reject a goal
rclcpp_action::GoalResponse ActionServerNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Analyze::Goal> goal)
// parameters: [
//   CallbackParameter("uuid", "rclcpp_action::GoalUUID"),
//   CallbackParameter("goal", "custom_interfaces::action::Analyze::Goal::ConstSharedPtr"),
// ]

// Cancel handler
rclcpp_action::CancelResponse ActionServerNode::handle_cancel(
    const std::shared_ptr<GoalHandleAnalyze> goal_handle)
// parameters: [CallbackParameter("goal_handle", "GoalHandleAnalyze::SharedPtr")]

// Accepted handler, queues the goal for the step timer
void ActionServerNode::handle_accepted(
    const std::shared_ptr<GoalHandleAnalyze> goal_handle)
// parameters: [CallbackParameter("goal_handle", "GoalHandleAnalyze::SharedPtr")]

// Client timer — sends the next goal
void ActionClientNode::timer_callback()
// parameters: []

// Feedback callback
void ActionClientNode::feedback_callback(
    GoalHandleAnalyze::SharedPtr goal_handle,
    const std::shared_ptr<const Analyze::Feedback> feedback)
// parameters: [
//   CallbackParameter("goal_handle", "GoalHandleAnalyze::SharedPtr"),
//   CallbackParameter("feedback", "custom_interfaces::action::Analyze::Feedback::ConstSharedPtr"),
// ]
```

Action servers register three distinct callbacks (goal, cancel, accepted), each with different parameter signatures. The extraction tool captures all three in `ActionServerInfo` with references to their respective `CallbackInfo` entries.

## Communication

```
ActionClientNode --[analyze: Analyze]--> ActionServerNode
ActionServerNode --[feedback: Analyze::Feedback]--> ActionClientNode
ActionServerNode --[result: Analyze::Result]--> ActionClientNode
```

## Run

```bash
docker compose run --rm dev ros2 launch action_example bringup.launch.py
```

## What to Expect

```
[INFO] [action_server]: Action server ready on 'analyze'
[INFO] [action_client]: Sending goal: num_samples=10, analysis_type='statistical'
[INFO] [action_server]: Received goal: num_samples=10, analysis_type='statistical'
[INFO] [action_client]: Feedback: 10% complete
...
[INFO] [action_client]: Feedback: 90% complete
[INFO] [action_client]: Result received: success=true
```

## Cancel a running goal

`cancel.launch.py` starts the server with `ActionCancelClientNode` (node `cancel_client`).
The client sends one goal of 20 samples and calls `async_cancel_goal` from its feedback
callback once it has seen sample 3. The server stops at the next `is_canceling()` check.

```bash
docker compose run --rm dev ros2 launch action_example cancel.launch.py
```

Expected client lines under stock ROS 2 (the server may stop one sample later than
shown). Under lfc the stop sample is the same in every run:

```
[INFO] [action_example.cancel_client]: Feedback: samples_processed=3, progress=15.0%
[INFO] [action_example.cancel_client]: Sending cancel request after samples_processed=3
[INFO] [action_example.cancel_client]: Cancel response: return_code=0, goals_canceling=1
[INFO] [action_example.cancel_client]: Result: code=CANCELED, success=false, results=[0, 1.1, 2.2, 3.3]
```

The same file runs under rcllf in one process and with one federate per node
(`LF_LAUNCH=cancel docker/lf_build.sh action_example`, add `FEDERATE=per-node`).
