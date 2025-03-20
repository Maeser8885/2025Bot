// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.*;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(Constants.DriveConstants.maxSpeed);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;
  boolean fieldRel;
  RobotConfig config;
  public final boolean visionDriveTest = false;
  public Vision vision;
  private Field2d m_field;

  public DriveSubsystem() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Units.feetToMeters(maximumSpeed));
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      SwerveModule[] modules = swerveDrive.getModules();
      for (SwerveModule m : modules) {
        m.getAngleMotor().setMotorBrake(true);
        m.getDriveMotor().setMotorBrake(true);
      }
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    if (visionDriveTest) {
      setupPhotonVision();
      swerveDrive.stopOdometryThread();
    }

    fieldRel = true;
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
    setupPathPlanner();
  }

  public void setupPathPlanner() {
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // gets pose
        this::resetPose, // resetOdometry
        this::getSpeeds, // MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelativeWithSpeeds(speeds), // Method that will drive the robot given ROBOT
                                                                        // RELATIVE ChassisSpeeds. Also optionally
                                                                        // outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(0.1, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.1, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    PathfindingCommand.warmupCommand().schedule();
  }

  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  public void drive(double translationX, double translationY, double angularRotationX, boolean isFieldRelative,
      double speedFactor) {
    swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
        translationX * swerveDrive.getMaximumChassisVelocity(),
        translationY * swerveDrive.getMaximumChassisVelocity()), speedFactor),
        Math.pow(angularRotationX, 3) * swerveDrive.getMaximumChassisAngularVelocity(),
        isFieldRelative,
        false);
  }

  public void driveRobotRelativeWithSpeeds(ChassisSpeeds speeds) {
    swerveDrive.setChassisSpeeds(speeds);
  }

  public Command switchFieldRel() {
    return runOnce(() -> {
      fieldRel = !fieldRel;
      if (fieldRel)
        swerveDrive.zeroGyro();
      SmartDashboard.putBoolean("IsFieldRelative", fieldRel);
    });
  }

  // change field relativity based on driver preference
  public Command getDriveCommand() {
    return this.run(() -> {
      drive(-RobotContainer.m_driverController.getY(), -RobotContainer.m_driverController.getX(),
          -RobotContainer.m_driverController.getTwist(), fieldRel,
          -RobotContainer.m_driverController.getThrottle() / 2 + 0.5);
    });
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d initPose) {
    swerveDrive.resetOdometry(initPose);
  }

  public ChassisSpeeds getSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public Command driveToPose(Pose2d poseTarget) {
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 5.6,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(1947));

    return AutoBuilder.pathfindToPose(
        poseTarget,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is It Field Relative?", fieldRel);
    m_field.setRobotPose(getPose());
    if (visionDriveTest) {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }
  }
}
