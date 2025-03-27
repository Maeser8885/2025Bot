package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.ReefTarget.ReefBranchSide;

public class OliviaAndCharlesControlScheme extends ControlScheme {

    @Override
    public String getName() {
        return "Olivia and Charles";
    }
    
    public void configureBindings(){

        //OLIVIA
        //l1
        m_xboxController.y().onTrue(new InstantCommand(() -> {

          elevatorSubsystem.setTarget(Constants.ElevatorConstants.L1Setpoint);
          grabberSubsystem.setTarget(Constants.GrabberConstants.L1Setpoint);
        }));
        //l2
        m_xboxController.b().onTrue(new InstantCommand(() -> {

            elevatorSubsystem.setTarget(Constants.ElevatorConstants.L2Setpoint);
            grabberSubsystem.setTarget(Constants.GrabberConstants.L2Setpoint);
        }));
        //l3
        m_xboxController.a().onTrue(new InstantCommand(() -> {

            elevatorSubsystem.setTarget(Constants.ElevatorConstants.L3Setpoint);
            grabberSubsystem.setTarget(Constants.GrabberConstants.L3Setpoint);
          }));
        //loading station
        m_xboxController.x().onTrue(new InstantCommand(() -> {

            elevatorSubsystem.setTarget(Constants.ElevatorConstants.intakeSetpoint);
            grabberSubsystem.setTarget(Constants.GrabberConstants.intakeSetpoint);
          }));

          //rotate wrist
          m_xboxController.leftTrigger().onTrue(new InstantCommand(() -> {
            grabberSubsystem.rotateGrabber();
          }));
          m_xboxController.rightTrigger().onTrue(new InstantCommand(() -> {
            grabberSubsystem.rotateGrabberB();
          }));

          elevatorSubsystem.setDefaultCommand(elevatorSubsystem.elevatorJoystickMoveCommand());
          grabberSubsystem.setDefaultCommand(grabberSubsystem.grabberMoveCommand());

          m_xboxController.povRight().onTrue(new InstantCommand(() -> {
            elevatorSubsystem.setTarget(Constants.ElevatorConstants.downSetpoint);
            grabberSubsystem.setTarget(Constants.GrabberConstants.downSetpoint);
          }));

          
    m_xboxController.povUp().toggleOnTrue(new InstantCommand(()->{grabberSubsystem.intake();}));
    m_xboxController.povUp().toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
    
    m_xboxController.povDown().toggleOnTrue(new InstantCommand(()->{grabberSubsystem.outtake();}));
    m_xboxController.povDown().toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
    //fast intake
    m_xboxController.povLeft().toggleOnTrue(new InstantCommand(()->{grabberSubsystem.fastOuttake();}));
    m_xboxController.povLeft().toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));

          ///CHARLES
          //dpad down - drives to nearest branch
        //dpad up - drives to furthest branch
        //dpad up-left - drives to topleft branch
        //dpad up-right - drives to topright branch
        //dpad down-left - drives to bottom left branch
        //dpad down-right - drives to bottom right branch
        //right trigger - Rightintake
        //left trigger - LeftIntake
        //Left bumper - processor
        //right bumper - cages
        //left stick - auto targets left branch
        //right stick - auto targets right branch
          m_logitechController.povDown().onTrue(driveSubsystem.driveToReefPosition(0));
    m_logitechController.povUp().onTrue(driveSubsystem.driveToReefPosition(3));
    m_logitechController.povUpLeft().onTrue(driveSubsystem.driveToReefPosition(4));
    m_logitechController.povUpRight().onTrue(driveSubsystem.driveToReefPosition(2));
    m_logitechController.povDownLeft().onTrue(driveSubsystem.driveToReefPosition(5));
    m_logitechController.povDownRight().onTrue(driveSubsystem.driveToReefPosition(1));

    m_logitechController.rightTrigger().onTrue(driveSubsystem.driveToRightIntake());
    m_logitechController.leftTrigger().onTrue(driveSubsystem.driveToLeftIntake());

    m_logitechController.rightBumper().onTrue(driveSubsystem.driveToCages());
    m_logitechController.leftBumper().onTrue(driveSubsystem.driveToProcesser());

            
    m_logitechController.leftBumper().toggleOnTrue(new InstantCommand(()->{grabberSubsystem.intake();}));
    m_logitechController.leftBumper().toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
    
    m_logitechController.rightBumper().toggleOnTrue(new InstantCommand(()->{grabberSubsystem.outtake();}));
    m_logitechController.rightBumper().toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));

    m_logitechController.leftStick().onTrue((reefTargeter.setBranchSide(ReefBranchSide.LEFT)
    .andThen(Commands.runOnce(() -> driveSubsystem.getDrive().field.getObject(
                                                 "target")
                                                                    .setPose(
                                                                        reefTargeter.getCoralTargetPose())))));

    m_logitechController.rightStick().onTrue(reefTargeter.setBranchSide(ReefBranchSide.RIGHT)
    .andThen(Commands.runOnce(() -> driveSubsystem.getDrive().field.getObject(
                                                 "target")
                                                                    .setPose(
                                                                        reefTargeter.getCoralTargetPose()))));

    m_logitechController.button(8).toggleOnTrue(driveSubsystem.switchFieldRel());
    
    m_driverController.button(3).toggleOnTrue(driveSubsystem.switchFieldRel());
    
        }
}
