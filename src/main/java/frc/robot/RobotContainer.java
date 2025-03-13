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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  public GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  
  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_xboxController = new CommandXboxController(1);
  PathPlannerAuto driveAuto;
  SequentialCommandGroup auto;

  public RobotContainer() {
    configureBindings();

    driveAuto = new PathPlannerAuto("Coral+Intake");
    auto = new SequentialCommandGroup(
      driveAuto,
      new DriveToApriltag(driveSubsystem, Vision.Cameras.CENTER_CAM, 7),
      new DepositCoral(elevatorSubsystem, grabberSubsystem, 4),
     // grabberSubsystem.getOuttakeCommand(),
      new WaitCommand(1)
      //grabberSubsystem.getStopCommand()
    );
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(driveSubsystem.getDriveCommand());
    //trigger
    m_driverController.button(1).toggleOnTrue(new InstantCommand(()->{grabberSubsystem.intake();}));
    m_driverController.button(1).toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
    //sideButton
    m_driverController.button(2).toggleOnTrue(new InstantCommand(()->{grabberSubsystem.outtake();}));
    m_driverController.button(2).toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
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
      new InstantCommand(() -> {
        elevatorSubsystem.setTarget(Constants.ElevatorConstants.downSetpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.downSetpoint);
    }));
    //12 = L1
    m_driverController.button(12).onTrue(
      new InstantCommand(() -> {
       
        elevatorSubsystem.setTarget(Constants.ElevatorConstants.L1Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L1Setpoint);})
      );
        //rotate grabber
    m_driverController.button(5).onTrue(new InstantCommand(()->{grabberSubsystem.rotateGrabber();}));
    m_driverController.button(6).onTrue(new InstantCommand(()->{grabberSubsystem.rotateGrabberB();}));

    m_driverController.button(5).onFalse(new InstantCommand(()->{grabberSubsystem.stopGrabber();}));
    m_driverController.button(6).onFalse(new InstantCommand(()->{grabberSubsystem.stopGrabber();}));

    //3 = change field relativity
    m_driverController.button(3).toggleOnTrue(driveSubsystem.switchFieldRel());
          //manual elbow rotation with xbox controller bumpers
    m_xboxController.leftBumper().onTrue(new InstantCommand(() -> {grabberSubsystem.rotateElbowB();}));
    m_xboxController.rightBumper().onTrue(new InstantCommand(() -> {grabberSubsystem.rotateElbow();}));
    //manual elevator movement with xbox controller triggers
    m_xboxController.leftTrigger().onTrue(new InstantCommand(()->{elevatorSubsystem.moveDown();}));
    m_xboxController.rightTrigger().onTrue(new InstantCommand(()->{elevatorSubsystem.moveUp();}));
        //xbox presets - y:L1 b:L2 a:L3 b:L4 dpadl:down dpadr:up 
    m_xboxController.y().onTrue(new InstantCommand(() -> {
       
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L1Setpoint);
    grabberSubsystem.setTarget(Constants.GrabberConstants.L1Setpoint);}));

  m_xboxController.b().onTrue(new InstantCommand(() -> {
    elevatorSubsystem.setTarget(Constants.ElevatorConstants.L2Setpoint);
    grabberSubsystem.setTarget(Constants.GrabberConstants.L2Setpoint);
  }));

  m_xboxController.a().onTrue(new InstantCommand(() -> {
    elevatorSubsystem.setTarget(Constants.ElevatorConstants.L3Setpoint);
    grabberSubsystem.setTarget(Constants.GrabberConstants.L3Setpoint);
  }));

  m_xboxController.x().onTrue(
    new InstantCommand(() -> {

      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L4Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L4Setpoint);
    })
  );

  m_xboxController.povLeft().onTrue(new InstantCommand(() -> {
    elevatorSubsystem.setTarget(Constants.ElevatorConstants.downSetpoint);
  grabberSubsystem.setTarget(Constants.GrabberConstants.downSetpoint);
}));

m_xboxController.povRight().onTrue(new InstantCommand(() -> {
  elevatorSubsystem.setTarget(Constants.ElevatorConstants.intakeSetpoint);
  grabberSubsystem.setTarget(Constants.GrabberConstants.intakeSetpoint);
}));

  }
  public Command getAutonomousCommand() {
    return auto;
  }
}
