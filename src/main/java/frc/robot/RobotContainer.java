// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DepositCoral;
import frc.robot.commands.DriveToApriltag;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;


public class RobotContainer {
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  public GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  
  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  PathPlannerAuto driveAuto;
  SequentialCommandGroup auto;

  public RobotContainer() {
    configureBindings();

    driveAuto = new PathPlannerAuto("Coral+Intake");
    auto = new SequentialCommandGroup(
      driveAuto,
      new DriveToApriltag(driveSubsystem, Vision.Cameras.CENTER_CAM, 7),
      new DepositCoral(elevatorSubsystem, grabberSubsystem, 4),
      grabberSubsystem.getOuttakeCommand(),
      new WaitCommand(1),
      grabberSubsystem.getStopCommand()
    );
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(driveSubsystem.getDriveCommand());
    //trigger
    m_driverController.button(1).toggleOnTrue(grabberSubsystem.getOuttakeCommand());
    m_driverController.button(1).toggleOnFalse(grabberSubsystem.getStopCommand());
    //sideButton
    m_driverController.button(2).toggleOnTrue(grabberSubsystem.getIntakeCommand());
    m_driverController.button(2).toggleOnFalse(grabberSubsystem.getStopCommand());
    //7 = intake
    m_driverController.button(7).onTrue(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.intakeSetpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.intakeSetpoint);
    }));
    //8 = L4
    m_driverController.button(8).onTrue(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L4Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L4Setpoint);
    }));
    //9 = L2
    m_driverController.button(9).onTrue(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L2Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L2Setpoint);
    }));
    //10 = L3
    m_driverController.button(10).onTrue(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L3Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L3Setpoint);
    }));
    //11 = DOWN
    m_driverController.button(11).onTrue(
      new InstantCommand(() -> {elevatorSubsystem.setTarget(Constants.ElevatorConstants.downSetpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.downSetpoint);}));
    //12 = L1
    m_driverController.button(12).onTrue(
      new InstantCommand(() -> {elevatorSubsystem.setTarget(Constants.ElevatorConstants.L1Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L1Setpoint);})
      );

    m_driverController.button(4).toggleOnTrue(grabberSubsystem.rotateGrabber());
    //3 = change field relativity
    m_driverController.button(3).toggleOnTrue(driveSubsystem.switchFieldRel());
  }


  public Command getAutonomousCommand() {
    return auto;
  }
}
