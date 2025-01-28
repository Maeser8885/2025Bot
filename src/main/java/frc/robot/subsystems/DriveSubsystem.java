// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import frc.robot.*;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

double maximumSpeed = Units.feetToMeters(Constants.DriveConstants.maxSpeed);
File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;
  public DriveSubsystem() {
    try{
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    }
    catch(Exception e){
      throw new RuntimeException(e);
    }
  }

  public Command getDriveCommand(double translationX, double translationY, double angularRotationX)
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(translationX * swerveDrive.getMaximumVelocity(),
                                          translationY * swerveDrive.getMaximumVelocity()),
                        angularRotationX * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  @Override
  public void periodic() {

  }

}
