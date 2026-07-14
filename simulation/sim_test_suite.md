# Simulation Test Suite: VLA-WAM Validation
# Goal: Mathematically verify the safety and stability of the control loop.

## Test Case 1: The "Wall" Test (Safety Veto)
**Setup**: Robot is placed 0.5m from a wall.
**VLA Input**: "Move forward to the wall."
**Expected Behavior**:
1. `vla_client_bridge` requests action $\rightarrow$ `mock_vla_server` returns `move_fwd`.
2. `predictive_governor` reads LiDAR $\rightarrow$ detects distance < 0.4m.
3. `predictive_governor` VETOES the action $\rightarrow$ publishes `stop`.
4. Robot remains stationary despite the VLA command.
**Success Metric**: Robot does not touch the wall.

## Test Case 2: The "Override" Test (Manual Priority)
**Setup**: Robot is moving forward in `vla` mode (`move_fwd`).
**VLA Input**: "Continue forward."
**User Action**: Press `joy_controller`'s mode button to cycle to `manual`, then push the stick left.
**Expected Behavior**:
1. `motor_driver` receives `joy_controller/mode` = `manual` and stops following `cmd_vel_final`.
2. `motor_driver` follows `cmd_vel_joy` instead.
3. Robot turns left immediately, ignoring the VLA forward command.
**Success Metric**: Zero latency in switching from Auto to Manual.

**Related, more severe case**: if the joystick is lost entirely (controller unplugged, `joy_controller` crashed) while in `vla` or `traditional` mode, `motor_driver` stops the robot outright rather than continuing to follow the autonomous source — verified by killing the `/joy` feed and confirming `/odom`'s twist goes to zero. See `jetbot_base`'s README for why this is a deliberate rule, not just a timeout side effect.

## Test Case 3: The "Latency Spike" Test (Stability)
**Setup**: Artificially introduce a 2-second sleep in the `mock_vla_server`.
**Expected Behavior**:
1. The robot should not "drift" or continue moving blindly.
2. The `predictive_governor` should either maintain the last safe state or trigger a timeout stop.
**Success Metric**: Robot stops or maintains safe distance during server downtime.
