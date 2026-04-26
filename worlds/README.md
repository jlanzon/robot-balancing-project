# Partial Single-Leg Support and Static Balance Recovery for a NAO Humanoid Robot in Webots

This project implements a static balance recovery behaviour for the NAO humanoid robot in Webots.

The robot starts in a stable two-foot stance, shifts its weight slightly to the left, enters a partial single-leg support posture, holds the posture briefly, and then returns to a stable two-foot stance without falling over.

The controller uses the NAO inertial unit to monitor roll and pitch values. These values are compared against the robot's initial baseline posture. Corrective movements are then applied to the ankle and hip joints to help the robot maintain balance during the movement sequence.

## Success Criterion

The project is considered successful if the robot completes the full movement sequence and returns to a stable two-foot stance while keeping roll and pitch error below 0.70 radians.

## Technologies Used

- Webots
- Python (My version was Python 3.10.6)
- NAO humanoid robot model
- Inertial unit sensor
- Motor control for ankle, hip, knee and shoulder joints
- CSV logging for evaluation

## Project Structure

```text
controllers/balance_controller/balance_controller.py
worlds/robot_balancing.wbt
results/weight_shift_balance_log.csv
```

## How to Run

- Open Webots.
- Open worlds/robot_balancing.wbt.
- Select the NAO robot.
- Confirm the controller is set to balance_controller.
- Press Run.
- The robot will perform the weight-shift balance recovery sequence.
- Results are then logged into weight_shift_balance_log.csv.

## Console Logs

The controller logs time, phase, roll, pitch, yaw, roll error, pitch error, correction values and upright status. This data is used to evaluate whether the robot remained stable during the movement sequence.
