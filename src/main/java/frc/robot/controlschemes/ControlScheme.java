package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import swervelib.SwerveInputStream;

public abstract class ControlScheme {

    DriveSubsystem driveSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    GrabberSubsystem grabberSubsystem;
    CommandJoystick m_driverController;
    CommandXboxController m_xboxController;
    CommandPS4Controller m_logitechController;
    public String name = "NO-NAME-ASSIGNED";


    public ControlScheme() {
        driveSubsystem = RobotContainer.instance.driveSubsystem;
        grabberSubsystem = RobotContainer.instance.grabberSubsystem;
        elevatorSubsystem = RobotContainer.instance.elevatorSubsystem;
        m_driverController = RobotContainer.m_driverController;
        m_xboxController = RobotContainer.m_xboxController;
        m_logitechController = RobotContainer.m_logitechController;
    }

    public final void configureUniversalBindings(){
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
Command driveFieldOrientedAngular = driveSubsystem.driveWithTheSpeeds(driveAngularVelocity);
  Command driveFieldOrientedDirectAngle = driveSubsystem.driveWithTheSpeeds(driveDirectAngle);
    if(RobotContainer.instance.driveChooser != null){
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
    default:
      driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);
      break;
  }}
  else{driveSubsystem.setDefaultCommand(driveFieldOrientedAngular);}
    }

    public void configureBindings(){
        //logitech buttons
        //  BTN_A = 1;
        //  BTN_B = 2;
        //  BTN_X = 3;
        //  BTN_Y = 4;
        //  BTN_LB = 5;
        //  BTN_RB = 6;
        //  BTN_BACK = 7;
        //  BTN_START = 8;
        //  BTN_LEFT_JOYSTICK = 9;
        //  BTN_RIGHT_JOYSTICK = 10;
    }
}
