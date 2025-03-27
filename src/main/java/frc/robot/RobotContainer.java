// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DepositCoral;
import frc.robot.commands.IntakeCoral;
import frc.robot.controlschemes.AbrarAndGavinControlScheme;
import frc.robot.controlschemes.ControlScheme;
import frc.robot.controlschemes.ExampleControlScheme;
import frc.robot.controlschemes.OliviaAndCharlesControlScheme;
import frc.robot.controlschemes.StandardControlScheme;
import frc.robot.subsystems.GrabberSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  public static RobotContainer instance = null;

  public ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public DriveSubsystem driveSubsystem = new DriveSubsystem();
  public GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  public ReefTarget reefTargeter = new ReefTarget();
  private final SendableChooser<Command> autoChooser;

  public ControlScheme defaultControlScheme;
  public List<ControlScheme> controlSchemes;

  private final SendableChooser<ControlScheme> m_controlsChooser = new SendableChooser<>();
  public ControlScheme controls;

  UsbCamera camera;
  RobotConfig config;

  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_xboxController = new CommandXboxController(1);
  public static final CommandXboxController m_logitechController = new CommandXboxController(2);
  public SendableChooser<String> driveChooser;

  public RobotContainer() {
    camera = CameraServer.startAutomaticCapture(0);
    camera.setFPS(30);
    instance = this;
    defaultControlScheme = new StandardControlScheme();
    controlSchemes = Arrays.asList(new ExampleControlScheme(), new OliviaAndCharlesControlScheme(), new AbrarAndGavinControlScheme());
    NamedCommands.registerCommand("DepositCoral", new DepositCoral(elevatorSubsystem, grabberSubsystem));
    NamedCommands.registerCommand("Go To Right Intake Station", driveSubsystem.driveToRightIntake());
    NamedCommands.registerCommand("Go To Left Intake Station", driveSubsystem.driveToLeftIntake());
    NamedCommands.registerCommand("Go To Beginning Of Reef", driveSubsystem.driveToReefPosition(0));
    NamedCommands.registerCommand("Go To Bottom-Left Of Reef", driveSubsystem.driveToReefPosition(1));
    NamedCommands.registerCommand("Go To Bottom-Right Of Reef", driveSubsystem.driveToReefPosition(5));
    NamedCommands.registerCommand("Intake Coral", new IntakeCoral(elevatorSubsystem, grabberSubsystem));
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("SingleCoralCenter", new PathPlannerAuto("SingleCoralCenter"));
    setupDashboard();
  }

  public void setupDashboard(){
    m_controlsChooser.setDefaultOption(defaultControlScheme.getName(), defaultControlScheme);
    for (ControlScheme controlScheme : controlSchemes) {m_controlsChooser.addOption(controlScheme.getName(), controlScheme);}
    SmartDashboard.putData("Control Scheme", m_controlsChooser);
    SmartDashboard.putData("Choose Autonomous Command", autoChooser);
    driveChooser = new SendableChooser<>();
    driveChooser.addOption("Joystick", "Joystick");
    driveChooser.addOption("Controller", "Controller");
    driveChooser.addOption("Richard Command", "Richard Command");
    driveChooser.setDefaultOption("Controller", "Controller");
    SmartDashboard.putData("Choose the Drive", driveChooser);
    
  }

  public void teleopInit(){
    controls = m_controlsChooser.getSelected();
    controls.configureUniversalBindings();
    controls.configureBindings();
  }

  public void teleopPeriodic(){
     SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveSubsystem.getDrive(),
                                                                () -> m_logitechController.getLeftY() * -1,
                                                                () -> m_logitechController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> m_logitechController.getRightX() * -1)
                                                            .deadband(0.2)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> m_logitechController.getRightX() * -1,
                                                                                             ()->m_logitechController.getRightY() * -1)
                                                           .headingWhile(true);
Command driveFieldOrientedAngular = driveSubsystem.driveWithTheSpeeds(driveAngularVelocity);
  Command driveFieldOrientedDirectAngle = driveSubsystem.driveWithTheSpeeds(driveDirectAngle);
    if(RobotContainer.instance.driveChooser != null){
  switch (driveChooser.getSelected()) {
    case "Joystick":
      driveSubsystem.setDefaultCommand(driveSubsystem.getDefaultCommand());
      break;
    case "Controller":
      driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);
      break;
    case "Richard Command":
      driveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
      break;
    default:
      driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);
      break;
  }}
  else{driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);}
    }

  

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
