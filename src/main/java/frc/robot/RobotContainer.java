// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
// import frc.robot.controlschemes.AbrarAndGavinControlScheme;
// import frc.robot.controlschemes.ExampleControlScheme;
// import frc.robot.controlschemes.IsaacAndLoganControlScheme;
//import frc.robot.subsystems.GrabberSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FirewoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Arrays;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  public static RobotContainer instance = null;

  // public ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public DriveSubsystem driveSubsystem = new DriveSubsystem();
  public FirewoodSubsystem firewoodSubsystem = new FirewoodSubsystem();
  public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  // // public ReefTarget reefTargeter = new ReefTarget();
  private final SendableChooser<Command> autoChooser;

  // public Vision vision;

  // UsbCamera camera;
  // UsbCamera camera2;
  // RobotConfig config;

  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_xboxController = new CommandXboxController(1);

  //public static final CommandXboxController m_logitechController = new CommandXboxController(2);
  public SendableChooser<String> driveChooser;

  public RobotContainer() {
    // camera2 = CameraServer.startAutomaticCapture(0);
    // camera = CameraServer.startAutomaticCapture(1);
    // camera.setFPS(30);
    // camera2.setFPS(30);
    instance = this;
    // controlSchemes = Arrays.asList(new ExampleControlScheme(), new
    // AbrarAndGavinControlScheme(), new IsaacAndLoganControlScheme());
    // NamedCommands.registerCommand("DepositCoral", new
    // DepositCoral(elevatorSubsystem, grabberSubsystem));
    // NamedCommands.registerCommand("Go To Right Intake Station",
    // driveSubsystem.driveToRightIntake());
    // NamedCommands.registerCommand("Go To Left Intake Station",
    // driveSubsystem.driveToLeftIntake());
    // NamedCommands.registerCommand("Go To Beginning Of Reef",
    // driveSubsystem.driveToReefPosition(0));
    // NamedCommands.registerCommand("Go To Bottom-Left Of Reef",
    // driveSubsystem.driveToReefPosition(1));
    // NamedCommands.registerCommand("Go To Bottom-Right Of Reef",
    // driveSubsystem.driveToReefPosition(5));
    // //NamedCommands.registerCommand("Intake Coral", new
    // IntakeCoral(elevatorSubsystem, grabberSubsystem));

    driveChooser = new SendableChooser<>();
    driveChooser.addOption("Joystick", "Joystick");
    driveChooser.addOption("Controller", "Controller");
    driveChooser.addOption("Richard Command", "Richard Command");
    autoChooser = AutoBuilder.buildAutoChooser();
    
    setupDashboard();
  }

  public void setupDashboard() {
    
    SmartDashboard.putData("Choose the Drive", driveChooser);

  }

  public void teleopInit() {

    configureUniversalBindings();
    configureBindings();
  }
  
  public void teleopPeriodic() {
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveSubsystem.getDrive(),
        () -> m_xboxController.getRawAxis(0) * 1,
        () -> m_xboxController.getRawAxis(1) * -1)
        .withControllerRotationAxis(() -> m_xboxController.getRawAxis(2) * -1)
        .deadband(0.2)
        .scaleTranslation(0.9)
        .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(() -> m_xboxController.getRawAxis(3) * 1,
            () -> m_xboxController.getRawAxis(2) * -1)
        .headingWhile(true);
    Command driveFieldOrientedAngular = driveSubsystem.driveWithTheSpeeds(driveAngularVelocity);
    Command driveFieldOrientedDirectAngle = driveSubsystem.driveWithTheSpeeds(driveDirectAngle);
    if (RobotContainer.instance.driveChooser != null) {
      switch (driveChooser.getSelected()) {
        case "Joystick":
          driveSubsystem.setDefaultCommand(driveSubsystem.getDefaultCommand());
          break;
          // autoChooser.setDefaultOption("SingleCoralCenter", new PathPlannerAuto("SingleCoralCenter"));
        case "Controller":
          driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);
          break;
        case "Richard Command":
          driveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
          break;
        default:
          driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);
          break;
      }
    } else {
      driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);
    }
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return new InstantCommand(() -> {
      driveSubsystem.lockModules();
    });
  }

  public final void configureUniversalBindings() {

    SwerveInputStream xboxDriveAngularVelocity = SwerveInputStream.of(driveSubsystem.getDrive(),
        () -> m_xboxController.getRawAxis(0) * 1,
        () -> m_xboxController.getRawAxis(1) * -1)
        .withControllerRotationAxis(() -> m_xboxController.getRawAxis(2) * -1)
        .deadband(0.2)
        .scaleTranslation(0.9)
        .allianceRelativeControl(true);

    SwerveInputStream xboxDriveDirectAngle = xboxDriveAngularVelocity.copy()
        .withControllerHeadingAxis(() -> m_xboxController.getRawAxis(3) * 1,
            () -> m_xboxController.getRawAxis(2) * -1)
        .headingWhile(true);

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveSubsystem.getDrive(),
        () -> m_xboxController.getRawAxis(0) * 1,
        () -> m_xboxController.getRawAxis(1) * -1)
        .withControllerRotationAxis(() -> m_xboxController.getRawAxis(2) * -1)
        .deadband(0.2)
        .scaleTranslation(0.9)
        .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(() -> m_xboxController.getRawAxis(3) * 1,
            () -> m_xboxController.getRawAxis(2) * -1)
        .headingWhile(true);
    Command driveFieldOrientedAngular = driveSubsystem.driveWithTheSpeeds(driveAngularVelocity);
    Command driveFieldOrientedDirectAngle = driveSubsystem.driveWithTheSpeeds(driveDirectAngle);
    if (RobotContainer.instance.driveChooser != null) {
      switch (RobotContainer.instance.driveChooser.getSelected()) {
        case "Joystick":
          driveSubsystem.setDefaultCommand(driveSubsystem.getDefaultCommand());
          break;
        case "Controller":
          driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);
          break;
        case "Richard Command":
          driveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
          break;
        case "Xbox Drive":
          driveSubsystem.setDefaultCommand(driveSubsystem.driveWithTheSpeeds(xboxDriveDirectAngle));

        default:
          driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);
          break;
      }
    } else {
      driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);
    }
  }

  public void configureBindings() {
    m_driverController.button(1).onTrue(shooterSubsystem.outtake());
    m_driverController.button(1).onFalse(shooterSubsystem.stop());

    m_xboxController.button(10).toggleOnTrue(driveSubsystem.switchFieldRel());

    m_driverController.button(5).onTrue(firewoodSubsystem.intakeFirewood());
    m_driverController.button(5).onFalse(firewoodSubsystem.stopFirewood());

    m_driverController.button(6).onTrue(firewoodSubsystem.outtakeFirewood());
    m_driverController.button(6).onFalse(firewoodSubsystem.stopFirewood());
  }
}
