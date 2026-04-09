# Swerve Drive Configuration Guide

## Prerequisites

- All swerve modules physically installed on the robot
- Robot code deployed with calibration readouts (SmartDashboard: `Calibration/<module> Raw Abs`)
- Access to Shuffleboard or SmartDashboard on the Driver Station laptop
- A straight edge or the frame rails for visual alignment

## Step 1: Set Encoder Offsets to Zero

In each module JSON file under `src/main/deploy/swerve/modules/`, temporarily set `"absoluteEncoderOffset"` to `0` for all four modules.

## Step 2: Physically Align All Wheels

1. With the robot **off**, point all four wheels straight forward.
2. Use the **bevel gear** as your reference — all bevel gears should face the **same direction** (left is the YAGSL convention, viewed from above).
3. The bevel gear is the key to avoiding a 180° ambiguity. A swerve wheel looks identical at 0° and 180°, but the bevel gear is asymmetric and always visible regardless of how the module is mounted on the chassis.

## Step 3: Read Raw Absolute Encoder Values

1. Deploy the code and connect to the robot.
2. While the robot is **disabled**, open SmartDashboard/Shuffleboard.
3. Read the `Calibration/<module name> Raw Abs` values for each module.
4. These raw values are your offsets.

## Step 4: Set Offsets in Module JSONs

Copy each module's raw absolute encoder value into its `"absoluteEncoderOffset"` field in the corresponding JSON file under `src/main/deploy/swerve/modules/`. Use the values exactly as read — they can be positive or negative.

Example:
```json
{
  "absoluteEncoderOffset": 172.81
}
```

## Step 5: Verify Offsets

1. Deploy the updated code.
2. The `Calibration/<module> Raw Abs` values should now read near `0` (or `360`) when wheels are pointed forward.

## Step 6: Verify Drive Directions

Use the calibration button bindings to test each axis independently:

| Button | Action |
|--------|--------|
| Y | Drive forward |
| A | Drive backward |
| X | Strafe left |
| B | Strafe right |
| LB | Rotate counter-clockwise |
| RB | Rotate clockwise |

### What to check:
- **Y (forward):** All 4 wheels should point the same way and spin the same direction.
- **X / B (strafe):** All wheels should turn 90° and spin together.
- **LB / RB (rotate):** Wheels should form an X pattern.

If a module drives **backward**, fix `"inverted"` on the drive motor in that module's JSON — don't adjust the offset.
If a module steers the **wrong direction**, fix `"inverted"` on the angle motor or `"absoluteEncoderInverted"` in that module's JSON.

## Other Controls

| Button | Action |
|--------|--------|
| Left stick | Drive (forward/back/strafe) |
| Right stick X | Rotate CW/CCW |
| Start | Toggle field-oriented / robot-oriented mode |
| Back | Zero gyro |

## Physical Properties Tuning

The file `src/main/deploy/swerve/modules/physicalproperties.json` controls motor limits, gear ratios, and physical characteristics. Here's how to set each value:

### Values you measure or look up from spec sheets

| Property | Description | How to determine |
|----------|-------------|-----------------|
| `optimalVoltage` | Battery voltage | Leave at **12**. Standard for FRC. |
| `robotMass` | Robot weight in pounds | Weigh the robot. Update if it changes significantly. |
| `drive.diameter` | Wheel diameter in inches | Measure wheels or check the spec sheet. |
| `drive.gearRatio` | Drive gear ratio | From the module spec sheet (e.g., SDS MK4i L2 = 6.75, L3 = 6.12). Must match your physical module. |
| `angle.gearRatio` | Steering gear ratio | From the module spec sheet. |

Getting the gear ratios and wheel diameter right is critical — if they're wrong, odometry and speed limits will be off.

### Values you tune by testing

| Property | Description | How to tune |
|----------|-------------|-------------|
| `wheelGripCoefficientOfFriction` | How grippy the wheels are | 1.0 is a reasonable default. Lower (e.g., 0.8) if the robot slides more than expected. Mostly affects simulation and trajectory planning. |
| `rampRate.drive` | Seconds from 0 to full drive power | Higher = smoother but slower response. Lower = snappier but can brown out or tip. Adjust based on driver preference. |
| `rampRate.angle` | Seconds from 0 to full steering power | Usually leave alone unless modules are oscillating. |

### Values you probably don't need to change

| Property | Description | Notes |
|----------|-------------|-------|
| `currentLimit.drive` | Drive motor current limit | Protects motors from burning out. 40-60A is typical for NEOs. If you trip breakers, lower it. |
| `currentLimit.angle` | Angle motor current limit | 20A is standard for steering. |
| `drive.factor` / `angle.factor` | Conversion factors | When set to 0, YAGSL auto-calculates from diameter and gear ratio. Leave at 0. |

## Other Swerve Configuration Files

All files are under `src/main/deploy/swerve/`.

### `swervedrive.json` — Robot-level config

| Property | Description | When to change |
|----------|-------------|----------------|
| `imu` | Gyro type and CAN ID | Set once to match hardware. Don't change unless you swap gyros. |
| `invertedIMU` | Invert gyro direction | If field-oriented driving goes the wrong way when you rotate the robot, flip to `true`. |
| `modules` | References to the four module JSON files | Order matters: front-left, front-right, back-left, back-right. If out of order, YAGSL will assign wrong module locations and kinematics will be wrong. |

### `controllerproperties.json` — Heading PID

| Property | Description | How to tune |
|----------|-------------|-------------|
| `angleJoystickRadiusDeadband` | How far you push the stick before heading correction kicks in (0.5 = 50%) | Leave alone unless heading feels twitchy or sluggish. |
| `heading.p` | Heading correction aggressiveness | If the robot oscillates (wobbles) when driving straight, lower it. If it's slow to correct heading drift, raise it. |
| `heading.i` | Heading steady-state error correction | Usually leave at 0. |
| `heading.d` | Heading damping | Helps reduce overshoot. Small values (0.01) are typical. |

### `modules/pidfproperties.json` — Module-level PID

| Property | Description | How to tune |
|----------|-------------|-------------|
| `drive.p` | Drive motor speed accuracy | Only matters in closed-loop mode. If wheels overshoot/oscillate, lower it. If sluggish, raise it. |
| `angle.p` | Steering motor angle accuracy | If modules jitter when holding position, lower it. If slow to turn, raise it. |
| `f` | Feed-forward | Usually 0 for position control. Can help with velocity control. |
| `iz` | I-zone (range where `i` is active) | Leave at 0 unless you add an `i` term. |

### Module files (`frontleft.json`, etc.) — Per-module config

| Property | Description | When to change |
|----------|-------------|----------------|
| `location` | Distance from robot center in inches (front/left positive, back/right negative) | Measure from robot center to each module center. Set once. |
| `absoluteEncoderOffset` | Encoder offset in degrees | Set during calibration (Steps 1-5). |
| `drive` / `angle` | Motor type and CAN ID | Set once to match hardware. |
| `encoder` | Absolute encoder type and ID | Set once to match hardware. |
| `inverted.drive` / `inverted.angle` | Motor inversions | Fix during drive direction verification (Step 6). |
| `absoluteEncoderInverted` | Invert encoder direction | If steering goes the wrong way after setting offsets. |

### Recommended tuning order

1. Encoder offsets (Steps 1-5)
2. Motor inversions (Step 6)
3. IMU inversion if needed
4. Angle PID (get modules holding position without jitter)
5. Drive PID (if using closed-loop)
6. Heading PID (last — fine-tune straight-line driving)

## References

- [YAGSL Swerve Modules](https://docs.yagsl.com/fundamentals/swerve-modules)
- [YAGSL Absolute Encoders](https://docs.yagsl.com/devices/absolute-encoders)
- [YAGSL Module Configuration](https://docs.yagsl.com/configuring-yagsl/configuration/swerve-module-configuration)
- [YAGSL Check Your Motors](https://docs.yagsl.com/bringing-up-swerve/check-your-motors)
