package frc.robot;

/**
 * Robot-wide constants that are shared across multiple subsystems.
 *
 * <p>Subsystem-specific constants live next to their subsystem to minimize merge conflicts
 * when multiple students work in parallel:
 * <ul>
 *   <li>{@code subsystems/DrivetrainConstants.java} — speed limits, slow mode</li>
 * </ul>
 *
 * <p>This file only contains values used by multiple files (controller ports, auto settings).
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kJoystickDeadband = 0.08;
  }
}
