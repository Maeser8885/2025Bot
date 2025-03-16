package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ExampleControlScheme extends ControlScheme {
    public ExampleControlScheme(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, CommandJoystick joystick, CommandXboxController xboxController) {
        super(driveSubsystem, elevatorSubsystem, grabberSubsystem);
    }

    public void configureBindings(){
        /*
         * Put all bindings here
         * 
         * 
         * 
         */
    }
}
