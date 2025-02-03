# Behavior Executive

The behavior executive reads which actions are active from the behavior tree and implements the behavior which these actions should perform and sets the status of the actions to SUCCESS, RUNNING, or FAILURE. It also sets the status of conditions as either SUCCESS or FAILURE.

A typical way of implementing the behavior for an action is the following in the 20 Hz timer callback:

```
if(action->is_active()){
  if(action->active_has_changed()){
    // This is only true when the when the action transitions between active/inactive
    // so this block of code will only run once whenever the action goes from being inactive to active.
    // You might put a service call here and then call action->set_success() or action->set_failure()
    // based on the result returned by the service call.
  }

  // Code here will get executed each iteration.
  // You might call action->set_running() while you are doing work here.
}
```