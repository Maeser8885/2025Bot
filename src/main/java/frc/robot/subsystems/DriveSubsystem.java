// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;
  boolean fieldRel;
  RobotConfig config;
  public DriveSubsystem() {
    try{
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Units.feetToMeters(maximumSpeed));
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      SwerveModule[] modules = swerveDrive.getModules();
      for(SwerveModule m: modules){
        m.getAngleMotor().setMotorBrake(true);
        m.getDriveMotor().setMotorBrake(true);
      }
      }
    catch(Exception e){
      throw new RuntimeException(e);
    }

    fieldRel = true;
    setUpAuto();
  }

  public void setUpAuto(){
    //config for auto
    try{
      config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = true; 
      //build auto
     AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry 
      this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> {
        if (enableFeedforward)
        {
          swerveDrive.drive(
              speeds,
              swerveDrive.kinematics.toSwerveModuleStates(speeds),
              feedforwards.linearForces()
                           );
        } else
        {
          swerveDrive.setChassisSpeeds(speeds);
        }
      }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. 
      new PPHolonomicDriveController(
              new PIDConstants(0.01, 0.0, 0.0), // drivePID
              new PIDConstants(0.01, 0.0, 0.0) // rotatePID
      ),
      config, 
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this 
);
    } catch (Exception e) {
      e.printStackTrace();
    } 
    
  }

  public Command drive(double translationX, double translationY, double angularRotationX, boolean isFieldRelative)
  {
    return this.run(() -> {
   swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX * swerveDrive.getMaximumChassisVelocity(),
                            translationY * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX, 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        isFieldRelative,
                        false);});
    }

    public Command switchFieldRel(){
   return runOnce(() ->{ fieldRel = !fieldRel;
    if(fieldRel)swerveDrive.zeroGyro(); SmartDashboard.putBoolean("IsFieldRelative", fieldRel);});
  }
//change field relativity based on driver preference
  public Command getDriveCommand(){
    return drive(-RobotContainer.m_driverController.getY(), -RobotContainer.m_driverController.getX(), -RobotContainer.m_driverController.getTwist(), fieldRel);
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d initialPose)
  {
    swerveDrive.resetOdometry(initialPose);
  }

    public ChassisSpeeds getCurrentSpeeds()
  {
    return swerveDrive.getRobotVelocity();
  }

}
