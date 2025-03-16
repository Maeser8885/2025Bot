package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public abstract class ControlScheme {

    DriveSubsystem driveSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    GrabberSubsystem grabberSubsystem;
    CommandJoystick m_driverController;
    CommandXboxController m_xboxController;


    public ControlScheme(DriveSubsystem _driveSubsystem, ElevatorSubsystem _elevatorSubsystem, GrabberSubsystem _grabberSubsystem) {
        driveSubsystem = _driveSubsystem;
        grabberSubsystem = _grabberSubsystem;
        elevatorSubsystem = _elevatorSubsystem;
        m_driverController = RobotContainer.m_driverController;
        m_xboxController = RobotContainer.m_xboxController;
    }

    public final void configureUniversalBindings(){
        driveSubsystem.setDefaultCommand(driveSubsystem.getDriveCommand());
    }

    public void configureBindings(){

    }
}
