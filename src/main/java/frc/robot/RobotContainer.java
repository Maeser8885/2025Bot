// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import swervelib.SwerveInputStream;
//import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;


public class RobotContainer {
  // subsystems and commands are defined here...
  DriveSubsystem driveSubsystem = new DriveSubsystem();


  ExampleSubsystem exampleSubsystem;
  
  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveSubsystem.getSwerveDrive(),
  () -> m_driverController.getY() * -1,
  () -> m_driverController.getX() * -1)
.withControllerRotationAxis(() -> m_driverController.getX())
.deadband(0)
.scaleTranslation(0.8)
.allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> m_driverController.getX(),
                                                                                             () -> m_driverController.getY())
                                                           .headingWhile(true);
  public RobotContainer() {
    configureBindings();
    exampleSubsystem = new ExampleSubsystem();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(driveSubsystem.driveFieldOriented(driveDirectAngle));

    
  }


  public Command getAutonomousCommand() {
    return new ExampleCommand(exampleSubsystem);
  }
}

