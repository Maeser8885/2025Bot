// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DepositCoral;
import frc.robot.commands.DriveDistance;
import frc.robot.controlschemes.ControlScheme;
import frc.robot.controlschemes.ExampleControlScheme;
import frc.robot.controlschemes.StandardControlScheme;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  public GrabberSubsystem grabberSubsystem = new GrabberSubsystem();


  private final SendableChooser<ControlScheme> m_controlsChooser = new SendableChooser<>();
  public ControlScheme controls;

  UsbCamera camera;
  
  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_xboxController = new CommandXboxController(1);
  SequentialCommandGroup auto;

  public RobotContainer() {
    controls.configureUniversalBindings();
    controls.configureBindings();
    camera = CameraServer.startAutomaticCapture(0);
    camera.setFPS(30);
    auto = new SequentialCommandGroup(
      new DriveDistance(driveSubsystem,-0.4 ,0.5, 1),
      new DepositCoral(elevatorSubsystem, grabberSubsystem)
    );
  }

  public void setupDashboard(){
    m_controlsChooser.setDefaultOption("Standard Control Scheme", new StandardControlScheme(driveSubsystem, elevatorSubsystem, grabberSubsystem, m_driverController, m_xboxController));
    m_controlsChooser.addOption("Drive Only", new ExampleControlScheme(driveSubsystem, elevatorSubsystem, grabberSubsystem, m_driverController, m_xboxController));
    SmartDashboard.putData("Control Scheme", m_controlsChooser);
  }

  public void teleopInit(){
    controls = m_controlsChooser.getSelected();
    controls.configureUniversalBindings();
    controls.configureBindings();
  }

  
  public Command getAutonomousCommand() {
    return auto;
  }
}
