# handle_matching_example

Reproduction systems for handle matching (an earlier binding matched ROS handles to
reactions loosely).

* `twin_sub`: `twin_publisher` publishes 0, 1, 2, ... on `twin` every 100 ms.
  `twin_subscriber` has two subscriptions on `twin` and logs `TWIN first <n>` and
  `TWIN second <n>`. Under rcllf both lines appear for every value, `first` before
  `second`.
* `lambda_timer`: a timer and a subscription whose callbacks are inline lambdas.
  The timer publishes 0, 1, 2, ... on `lambda_count` every 100 ms and logs
  `LAMBDA tick <n>`. The subscription listens to `twin`, which `twin_publisher`
  feeds with 0, 1, 2, ..., and logs `LAMBDA got <n>`. The analyzer names the
  callbacks `lambda_timer_0` and `lambda_subscription_0` after their creation order,
  and the binding matches the handles by that order.
* `opaque_callback`: a timer whose callback is a `std::function` member, so its body
  is not visible where the timer is created. `colcon lf-analyze` reports RCLLF-H007
  (error) naming the argument `tick_`, and `colcon lf-generate` refuses the system.
