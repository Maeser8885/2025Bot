package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;

import java.io.File;

/**
 * DrivetrainSubsystem controls the swerve drive — the four independently steerable wheel modules
 * that let the robot move in any direction while facing any direction.
 *
 * <p>In FRC, a "subsystem" represents a physical mechanism on the robot. This subsystem owns
 * all four swerve modules, the gyroscope, and the math that coordinates them. It uses YAGSL
 * (Yet Another Generic Swerve Library) to handle the complex swerve kinematics, so you don't
 * have to write the swerve math yourself — YAGSL reads JSON config files from the
 * {@code src/main/deploy/swerve/} directory and sets everything up automatically.
 *
 * <p>This subsystem supports two drive modes:
 * <ul>
 *   <li><b>Field-oriented</b> (default) — "forward" on the joystick always means the same
 *       direction on the field, regardless of which way the robot is facing. The gyro
 *       compensates for robot heading.</li>
 * </ul>
 */
public class DrivetrainSubsystem extends SubsystemBase {
    // Max speeds — start conservative, increase as drivers get comfortable
    public static final double kMaxSpeedMetersPerSecond = .5;
    public static final double kMaxAngularSpeedRadiansPerSecond = .25 * Math.PI; // 1 rotation/sec
    // Slow mode multiplier (hold bumper)
    public static final double kSlowModeMultiplier = 0.25;
    // Field-oriented drive enabled by default
    public static final boolean kFieldOrientedDefault = true;


    private final SwerveDrive swerveDrive;
    private boolean fieldOriented = kFieldOrientedDefault;

    public DrivetrainSubsystem() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(kMaxSpeedMetersPerSecond);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize swerve drive from JSON config", e);
        }
    }

    /**
     * Drive the robot with translation and rotation inputs.
     *
     *  Percent range: 0 - 1
     */
    public void drive(double forwardPercent, double leftPercent, double rotationPercent) {
        swerveDrive.drive(
                new Translation2d(forwardPercent * kMaxSpeedMetersPerSecond, leftPercent * kMaxSpeedMetersPerSecond),
                rotationPercent * kMaxAngularSpeedRadiansPerSecond,
                fieldOriented,
                false);
    }

    /**
     * Zero the gyro heading. Call this to re-zero field-oriented drive.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Toggle between field-oriented and robot-oriented drive modes.
     */
    public void toggleFieldOriented() {
        fieldOriented = !fieldOriented;
    }

    /**
     * Returns true if currently in field-oriented mode.
     */
    public boolean isFieldOriented() {
        return fieldOriented;
    }

    /**
     * Returns a command that immediately stops the drivetrain.
     */
    public Command stopCommand() {
        return runOnce(() -> drive(0, 0, 0));
    }

    /**
     * Expose the underlying SwerveDrive (e.g. for reading encoder values).
     */
    public void publishStats() {
        SmartDashboard.putBoolean("Drive/Field Oriented", fieldOriented);
        SmartDashboard.putNumber("Drive/Gyro Heading", swerveDrive.getYaw().getDegrees());
        SmartDashboard.putString("Drive/Pose", swerveDrive.getPose().getTranslation().toString());

        for (SwerveModule module : swerveDrive.getModules()) {
            String name = module.getConfiguration().name;
            double raw = module.getRawAbsolutePosition();
            SmartDashboard.putNumber("Calibration/" + name + " Raw Abs", raw);
        }

    }

    @Override
    public void periodic() {
        publishStats();
    }
}
