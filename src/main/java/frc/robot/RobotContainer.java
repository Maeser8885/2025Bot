// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;


public class RobotContainer {
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ExampleSubsystem exampleSubsystem;
  
  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);


  public RobotContainer() {
    configureBindings();
    exampleSubsystem = new ExampleSubsystem();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(driveSubsystem.getDriveCommand());
    
  }


  public Command getAutonomousCommand() {
    return new ExampleCommand(exampleSubsystem);
  }
}
