// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DepositCoral;
import frc.robot.subsystems.GrabberSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  public GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  private final SendableChooser<Command> autoChooser;

  UsbCamera camera;
  RobotConfig config;

  public static final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_xboxController = new CommandXboxController(1);
  public static final CommandPS4Controller m_logitechController = new CommandPS4Controller(2);
  SendableChooser<String> driveChooser;

  public RobotContainer() {
    configureBindings();
    camera = CameraServer.startAutomaticCapture(0);
    camera.setFPS(30);
    setupPathPlanner();
    NamedCommands.registerCommand("DepositCoral", new DepositCoral(elevatorSubsystem, grabberSubsystem));
    autoChooser = AutoBuilder.buildAutoChooser();

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

  public Command driveToLeftIntake() {
    if (PoseAlliances.shouldFlip()) {
      return driveSubsystem.driveToPose(PoseAlliances.flip(Constants.FieldConstants.leftIntake));
    } else {
      return driveSubsystem.driveToPose(Constants.FieldConstants.leftIntake);
    }
  }

  public Command driveToRightIntake() {
    if (PoseAlliances.shouldFlip()) {
      return driveSubsystem.driveToPose(PoseAlliances.flip(Constants.FieldConstants.rightIntake));
    } else {
      return driveSubsystem.driveToPose(Constants.FieldConstants.rightIntake);
    }
  }

  public Command driveToProcesser() {
    if (PoseAlliances.shouldFlip()) {
      return driveSubsystem.driveToPose(PoseAlliances.flip(Constants.FieldConstants.Processer));
    } else {
      return driveSubsystem.driveToPose(Constants.FieldConstants.Processer);
    }
  }

  // pos starting from 0 = driverstation going clockwise
  public Command driveToReefPosition(int pos) {
    if (PoseAlliances.shouldFlip()) {
      return driveSubsystem.driveToPose(PoseAlliances.flip(Constants.FieldConstants.reefPositions[pos]));
    } else {
      return driveSubsystem.driveToPose(Constants.FieldConstants.reefPositions[pos]);
    }
  }

  private void configureBindings() {
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveSubsystem.getDrive(),
                                                                () -> m_logitechController.getLeftY() * -1,
                                                                () -> m_logitechController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> m_logitechController.getRightX() * -1)
                                                            .deadband(0.2)
                                                            .scaleTranslation(0.9)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> m_logitechController.getRightX() * -1,
                                                                                             ()->m_logitechController.getRightY() * -1)
                                                           .headingWhile(true);
Command driveFieldOrientedAngular = driveSubsystem.driveFieldOrientedSpeeds(driveAngularVelocity);
  Command driveFieldOrientedDirectAngle = driveSubsystem.driveFieldOrientedSpeeds(driveDirectAngle);
    if(driveChooser != null){
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

    //trigger
    m_driverController.button(1).toggleOnTrue(new InstantCommand(()->{grabberSubsystem.intake();}));
    m_driverController.button(1).toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
    //sideButton
    m_driverController.button(2).toggleOnTrue(new InstantCommand(()->{grabberSubsystem.outtake();}));
    m_driverController.button(2).toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
    //fast intake
    m_driverController.button(5).toggleOnTrue(new InstantCommand(()->{grabberSubsystem.fastOuttake();}));
    m_driverController.button(5).toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
    //7 = intake
    m_driverController.button(7).onTrue(driveToLeftIntake().alongWith(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.intakeSetpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.intakeSetpoint);
    })));
    // 8 = L4
    m_driverController.button(8).onTrue(new InstantCommand(() -> {

      elevatorSubsystem.setTarget(Constants.ElevatorConstants.downSetpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.pSetpoint);
    }));
    //9 = L2
    m_driverController.button(9).onTrue(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L2Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L2Setpoint);
    }));
    // 10 = L3
    m_driverController.button(10).onTrue(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L3Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L3Setpoint);
    }));
    // 11 = DOWN
    m_driverController.button(11).onTrue(
        new InstantCommand(() -> {
          elevatorSubsystem.setTarget(Constants.ElevatorConstants.downSetpoint);
          grabberSubsystem.setTarget(Constants.GrabberConstants.downSetpoint);
        }));
    // 12 = L1
    m_driverController.button(12).onTrue(
        new InstantCommand(() -> {

          elevatorSubsystem.setTarget(Constants.ElevatorConstants.L1Setpoint);
          grabberSubsystem.setTarget(Constants.GrabberConstants.L1Setpoint);
        }));
    // rotate grabber
    m_xboxController.leftTrigger().onTrue(new InstantCommand(() -> {
      grabberSubsystem.rotateGrabber();
    }));
    m_xboxController.rightTrigger().onTrue(new InstantCommand(() -> {
      grabberSubsystem.rotateGrabberB();
    }));

    m_xboxController.leftTrigger().onFalse(new InstantCommand(() -> {
      grabberSubsystem.stopGrabber();
    }));
    m_xboxController.rightTrigger().onFalse(new InstantCommand(() -> {
      grabberSubsystem.stopGrabber();
    }));

    // 3 = change field relativity
    m_driverController.button(3).toggleOnTrue(driveSubsystem.switchFieldRel());
    // manual elbow rotation with xbox controller bumpers
    m_xboxController.povUp().onTrue(new InstantCommand(() -> {
      grabberSubsystem.rotateElbowB();
    }));
    m_xboxController.povDown().onTrue(new InstantCommand(() -> {
      grabberSubsystem.rotateElbow();
    }));

    m_xboxController.povUp().onFalse(new InstantCommand(() -> {
      grabberSubsystem.stopElbow();
    }));
    m_xboxController.povDown().onFalse(new InstantCommand(() -> {
      grabberSubsystem.stopElbow();
    }));
    // manual elevator movement with xbox controller triggers
    m_xboxController.rightStick().onTrue(new InstantCommand(() -> {
      elevatorSubsystem.moveDown();
    }));
    m_xboxController.leftStick().onTrue(new InstantCommand(() -> {
      elevatorSubsystem.moveUp();
    }));

    m_xboxController.rightStick().onFalse(new InstantCommand(() -> {
      elevatorSubsystem.stop();
    }));
    m_xboxController.leftStick().onFalse(new InstantCommand(() -> {
      elevatorSubsystem.stop();
    }));

    // xbox presets - y:L1 b:L2 a:L3 x:L4 dpadl:down dpadr:up
    m_xboxController.y().onTrue(new InstantCommand(() -> {

      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L1Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L1Setpoint);
    }));

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
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.intakeSetpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.intakeSetpoint);
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
    return autoChooser.getSelected();
  }
}
