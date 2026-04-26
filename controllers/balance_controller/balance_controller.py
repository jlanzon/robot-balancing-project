from controller import Robot
import csv
import math
import os

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print("Weight-shift balance controller started")

imu = robot.getDevice("inertial unit")

if imu is None:
    print('ERROR: Device named "inertial unit" was not found.')
    exit()

imu.enable(timestep)
print("InertialUnit found and enabled.")

motor_names = [
    "LAnklePitch",
    "RAnklePitch",
    "LHipPitch",
    "RHipPitch",
    "LKneePitch",
    "RKneePitch",
    "LAnkleRoll",
    "RAnkleRoll",
    "LHipRoll",
    "RHipRoll",
    "LShoulderPitch",
    "RShoulderPitch",
    "LShoulderRoll",
    "RShoulderRoll",
    "LElbowRoll",
    "RElbowRoll"
]

motors = {}

for name in motor_names:
    motor = robot.getDevice(name)

    if motor is None:
        print(f"Missing motor: {name}")
    else:
        motors[name] = motor
        print(f"Found motor: {name}")

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
results_dir = os.path.join(project_root, "results")
os.makedirs(results_dir, exist_ok=True)

log_path = os.path.join(results_dir, "weight_shift_balance_log.csv")
log_file = open(log_path, "w", newline="")
writer = csv.writer(log_file)

writer.writerow([
    "time_seconds",
    "phase",
    "roll",
    "pitch",
    "yaw",
    "roll_error",
    "pitch_error",
    "roll_correction",
    "pitch_correction",
    "status"
])

print(f"Logging results to: {log_path}")

PITCH_GAIN = 0.35
ROLL_GAIN = 0.35
MAX_CORRECTION = 0.16
FALL_THRESHOLD = 0.70

baseline_roll = None
baseline_pitch = None
counter = 0

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def smoothstep(x):
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)

def lerp(a, b, x):
    return a + (b - a) * x

def set_motor(name, value):
    if name in motors:
        motors[name].setPosition(value)

def interpolate_pose(start_pose, end_pose, progress):
    blend = smoothstep(progress)
    pose = {}

    all_joints = set(start_pose.keys()) | set(end_pose.keys())

    for joint in all_joints:
        start_value = start_pose.get(joint, 0.0)
        end_value = end_pose.get(joint, 0.0)
        pose[joint] = lerp(start_value, end_value, blend)

    return pose

neutral_pose = {
    "LAnklePitch": 0.00,
    "RAnklePitch": 0.00,
    "LHipPitch": 0.00,
    "RHipPitch": 0.00,
    "LKneePitch": 0.05,
    "RKneePitch": 0.05,
    "LAnkleRoll": 0.00,
    "RAnkleRoll": 0.00,
    "LHipRoll": 0.00,
    "RHipRoll": 0.00,
    "LShoulderPitch": 1.10,
    "RShoulderPitch": 1.10,
    "LShoulderRoll": 0.20,
    "RShoulderRoll": -0.20,
    "LElbowRoll": -0.50,
    "RElbowRoll": 0.50
}

shift_left_pose = {
    "LAnklePitch": 0.02,
    "RAnklePitch": 0.02,
    "LHipPitch": -0.03,
    "RHipPitch": -0.03,
    "LKneePitch": 0.08,
    "RKneePitch": 0.08,
    "LAnkleRoll": -0.10,
    "RAnkleRoll": -0.10,
    "LHipRoll": 0.12,
    "RHipRoll": 0.12,
    "LShoulderPitch": 1.00,
    "RShoulderPitch": 1.00,
    "LShoulderRoll": 0.50,
    "RShoulderRoll": -0.10,
    "LElbowRoll": -0.70,
    "RElbowRoll": 0.40
}

#change back later - looks weird 
partial_single_leg_pose = {
    "LAnklePitch": 0.03,
    "RAnklePitch": -0.08,
    "LHipPitch": -0.04,
    "RHipPitch": -0.18,
    "LKneePitch": 0.10,
    "RKneePitch": 0.32,
    "LAnkleRoll": -0.12,
    "RAnkleRoll": -0.08,
    "LHipRoll": 0.14,
    "RHipRoll": 0.08,
    "LShoulderPitch": 0.95,
    "RShoulderPitch": 0.95,
    "LShoulderRoll": 0.65,
    "RShoulderRoll": -0.05,
    "LElbowRoll": -0.80,
    "RElbowRoll": 0.35
}

return_pose = neutral_pose

def get_phase_and_pose(t):
    if t < 2.0:
        return "stable_two_foot_start", neutral_pose

    if 2.0 <= t < 4.0:
        progress = (t - 2.0) / 2.0
        return "shifting_weight_left", interpolate_pose(neutral_pose, shift_left_pose, progress)

    if 4.0 <= t < 6.0:
        progress = (t - 4.0) / 2.0
        return "partial_single_leg_support", interpolate_pose(shift_left_pose, partial_single_leg_pose, progress)

    if 6.0 <= t < 8.0:
        return "holding_partial_single_leg_support", partial_single_leg_pose

    if 8.0 <= t < 11.0:
        progress = (t - 8.0) / 3.0
        return "returning_to_two_foot_stance", interpolate_pose(partial_single_leg_pose, return_pose, progress)

    return "stable_two_foot_finish", return_pose

while robot.step(timestep) != -1:
    counter += 1
    time_seconds = robot.getTime()

    roll, pitch, yaw = imu.getRollPitchYaw()

    if baseline_roll is None:
        baseline_roll = roll
        baseline_pitch = pitch
        print(f"Baseline set: Roll={baseline_roll:.3f}, Pitch={baseline_pitch:.3f}")

    roll_error = roll - baseline_roll
    pitch_error = pitch - baseline_pitch

    pitch_correction = clamp(-pitch_error * PITCH_GAIN, -MAX_CORRECTION, MAX_CORRECTION)
    roll_correction = clamp(-roll_error * ROLL_GAIN, -MAX_CORRECTION, MAX_CORRECTION)

    phase, base_pose = get_phase_and_pose(time_seconds)

    leg_pitch_scale = 1.0
    leg_roll_scale = 1.0

    set_motor("LAnklePitch", base_pose.get("LAnklePitch", 0.0) + pitch_correction * leg_pitch_scale)
    set_motor("RAnklePitch", base_pose.get("RAnklePitch", 0.0) + pitch_correction * leg_pitch_scale)

    set_motor("LHipPitch", base_pose.get("LHipPitch", 0.0) - pitch_correction * 0.7)
    set_motor("RHipPitch", base_pose.get("RHipPitch", 0.0) - pitch_correction * 0.7)

    set_motor("LKneePitch", base_pose.get("LKneePitch", 0.0))
    set_motor("RKneePitch", base_pose.get("RKneePitch", 0.0))

    set_motor("LAnkleRoll", base_pose.get("LAnkleRoll", 0.0) + roll_correction * leg_roll_scale)
    set_motor("RAnkleRoll", base_pose.get("RAnkleRoll", 0.0) + roll_correction * leg_roll_scale)

    set_motor("LHipRoll", base_pose.get("LHipRoll", 0.0) - roll_correction * 0.7)
    set_motor("RHipRoll", base_pose.get("RHipRoll", 0.0) - roll_correction * 0.7)

    set_motor("LShoulderPitch", base_pose.get("LShoulderPitch", 1.0))
    set_motor("RShoulderPitch", base_pose.get("RShoulderPitch", 1.0))

    set_motor("LShoulderRoll", base_pose.get("LShoulderRoll", 0.2) + roll_correction)
    set_motor("RShoulderRoll", base_pose.get("RShoulderRoll", -0.2) + roll_correction)

    set_motor("LElbowRoll", base_pose.get("LElbowRoll", -0.5))
    set_motor("RElbowRoll", base_pose.get("RElbowRoll", 0.5))

    if abs(roll_error) > FALL_THRESHOLD or abs(pitch_error) > FALL_THRESHOLD:
        status = "unstable_or_fallen"
    else:
        status = "upright"

    writer.writerow([
        round(time_seconds, 3),
        phase,
        round(roll, 4),
        round(pitch, 4),
        round(yaw, 4),
        round(roll_error, 4),
        round(pitch_error, 4),
        round(roll_correction, 4),
        round(pitch_correction, 4),
        status
    ])

    log_file.flush()

    if counter % 20 == 0:
        print(
            f"t={time_seconds:.2f}s | "
            f"phase={phase} | "
            f"roll_error={roll_error:.3f}, "
            f"pitch_error={pitch_error:.3f}, "
            f"roll_corr={roll_correction:.3f}, "
            f"pitch_corr={pitch_correction:.3f}, "
            f"status={status}"
        )

log_file.close()