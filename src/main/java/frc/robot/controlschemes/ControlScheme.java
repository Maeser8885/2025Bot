package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ReefTarget;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public abstract class ControlScheme {

    DriveSubsystem driveSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    GrabberSubsystem grabberSubsystem;
    ReefTarget reefTargeter;
    CommandJoystick m_driverController;
    CommandXboxController m_xboxController;
    CommandXboxController m_logitechController;

    public ControlScheme() {
        driveSubsystem = RobotContainer.instance.driveSubsystem;
        grabberSubsystem = RobotContainer.instance.grabberSubsystem;
        elevatorSubsystem = RobotContainer.instance.elevatorSubsystem;
        reefTargeter = RobotContainer.instance.reefTargeter;
        m_driverController = RobotContainer.m_driverController;
        m_xboxController = RobotContainer.m_xboxController;
        m_logitechController = RobotContainer.m_logitechController;
    }

    public abstract String getName();

    public final void configureUniversalBindings(){
       
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
