// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;


public class RobotContainer {
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ExampleSubsystem exampleSubsystem;
  
  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);

   public static enum ElevatorPositions {
    DOWN,
    INTAKE,
    L1,
    L2,
    L3,
    L4
  }
  public static enum GrabberPositions {
    DOWN,
    INTAKE,
    L1,
    L2,
    L3,
    L4
  }
  public static ElevatorPositions m_elevatorPosition = ElevatorPositions.DOWN;
  public static Map<ElevatorPositions,Double> m_elevatorPositions = Map.ofEntries(
          Map.entry(ElevatorPositions.DOWN,Constants.ElevatorConstants.downSetpoint),
          Map.entry(ElevatorPositions.INTAKE,Constants.ElevatorConstants.intakeSetpoint),
          Map.entry(ElevatorPositions.L1,Constants.ElevatorConstants.L1Setpoint),
          Map.entry(ElevatorPositions.L2,Constants.ElevatorConstants.L2Setpoint),
          Map.entry(ElevatorPositions.L3,Constants.ElevatorConstants.L3Setpoint),
          Map.entry(ElevatorPositions.L4,Constants.ElevatorConstants.L4Setpoint)
  );
  public static GrabberPositions m_GrabberPosition = GrabberPositions.DOWN;
  public static Map<GrabberPositions,Double> m_GrabberPositions = Map.ofEntries(
          Map.entry(GrabberPositions.DOWN,Constants.GrabberConstants.downSetpoint),
          Map.entry(GrabberPositions.INTAKE,Constants.GrabberConstants.intakeSetpoint),
          Map.entry(GrabberPositions.L1,Constants.GrabberConstants.L1Setpoint),
          Map.entry(GrabberPositions.L2,Constants.GrabberConstants.L2Setpoint),
          Map.entry(GrabberPositions.L3,Constants.GrabberConstants.L3Setpoint),
          Map.entry(GrabberPositions.L4,Constants.GrabberConstants.L4Setpoint)
  );

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
