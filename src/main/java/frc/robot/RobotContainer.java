// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DepositCoral;
import frc.robot.controlschemes.ControlScheme;
import frc.robot.controlschemes.ExampleControlScheme;
import frc.robot.controlschemes.StandardControlScheme;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  public static RobotContainer instance = null;

  public ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public DriveSubsystem driveSubsystem = new DriveSubsystem();
  public GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  private final SendableChooser<Command> autoChooser;

  public ControlScheme defaultControlScheme = new StandardControlScheme();
  public List<ControlScheme> controlSchemes = Arrays.asList(new ExampleControlScheme());

  private final SendableChooser<ControlScheme> m_controlsChooser = new SendableChooser<>();
  public ControlScheme controls;

  UsbCamera camera;
  RobotConfig config;

  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_xboxController = new CommandXboxController(1);
  public static final CommandPS4Controller m_logitechController = new CommandPS4Controller(2);
  public SendableChooser<String> driveChooser;

  public RobotContainer() {
    camera = CameraServer.startAutomaticCapture(0);
    camera.setFPS(30);
    setupDashboard();
    setupPathPlanner();
    NamedCommands.registerCommand("DepositCoral", new DepositCoral(elevatorSubsystem, grabberSubsystem));
    autoChooser = AutoBuilder.buildAutoChooser();
  }

  public void setupDashboard(){
    m_controlsChooser.setDefaultOption(defaultControlScheme.name, defaultControlScheme);
    for (ControlScheme controlScheme : controlSchemes) {m_controlsChooser.addOption(controlScheme.name, controlScheme);}
    SmartDashboard.putData("Control Scheme", m_controlsChooser);
    SmartDashboard.putData("Choose Autonomous Command", autoChooser);
    driveChooser = new SendableChooser<>();
    driveChooser.addOption("Joystick", "Joystick");
    driveChooser.addOption("Controller", "Controller");
    driveChooser.addOption("Richard Command", "Richard Command");
    SmartDashboard.putData(driveChooser);
  }

  public void setupPathPlanner() {
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        driveSubsystem::getPose, // gets pose
        driveSubsystem::resetPose, // resetOdometry
        driveSubsystem::getSpeeds, // MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveSubsystem.driveRobotRelativeWithSpeeds(speeds), // Method that will drive the
                                                                                       // robot given ROBOT RELATIVE
                                                                                       // ChassisSpeeds. Also optionally
                                                                                       // outputs individual module
                                                                                       // feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(0.1, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.1, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        driveSubsystem);
  }

  public void teleopInit(){
    controls = m_controlsChooser.getSelected();
    controls.configureUniversalBindings();
    controls.configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
