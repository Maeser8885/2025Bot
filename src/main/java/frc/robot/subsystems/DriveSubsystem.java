// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.*;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Translation2d;

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
  }

  
  public void drive(double translationX, double translationY, double angularRotationX, boolean isFieldRelative, double speedFactor){
   swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX * swerveDrive.getMaximumChassisVelocity(),
                            translationY * swerveDrive.getMaximumChassisVelocity()), (speedFactor)),
                        Math.pow(angularRotationX, 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        isFieldRelative,
                        false);
    }

    public Command switchFieldRel(){
   return runOnce(() ->{ fieldRel = !fieldRel;
    if(fieldRel)swerveDrive.zeroGyro(); SmartDashboard.putBoolean("IsFieldRelative", fieldRel);});
  }
//change field relativity based on driver preference
  public Command getDriveCommand(){
    return this.run(()->{drive(-RobotContainer.m_driverController.getY(), -RobotContainer.m_driverController.getX(), -RobotContainer.m_driverController.getTwist(), fieldRel, -RobotContainer.m_driverController.getThrottle()/2 + 0.5);});
  }


}
