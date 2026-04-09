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

## References

- [YAGSL Swerve Modules](https://docs.yagsl.com/fundamentals/swerve-modules)
- [YAGSL Absolute Encoders](https://docs.yagsl.com/devices/absolute-encoders)
- [YAGSL Module Configuration](https://docs.yagsl.com/configuring-yagsl/configuration/swerve-module-configuration)
- [YAGSL Check Your Motors](https://docs.yagsl.com/bringing-up-swerve/check-your-motors)
