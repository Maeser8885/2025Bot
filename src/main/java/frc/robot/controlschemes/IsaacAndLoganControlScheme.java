// package frc.robot.controlschemes;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.Constants;
// import frc.robot.ReefTarget.ReefBranchSide;

// public class IsaacAndLoganControlScheme extends ControlScheme {

//     @Override
//     public String getName() {
//         return "Isaac and Logan";
//     }}

//   /*   public void configureBindings(){
//         elevatorSubsystem.setDefaultCommand(elevatorSubsystem.logitechElevatorCommand());
//         grabberSubsystem.setDefaultCommand(grabberSubsystem.logitechGrabberMoveCommand());

//         //l1
//         m_logitechController.y().onTrue(new InstantCommand(() -> {

//           elevatorSubsystem.setTarget(Constants.ElevatorConstants.L1Setpoint);
//           grabberSubsystem.setTarget(Constants.GrabberConstants.L1Setpoint);
//         }));
//         //l2
//         m_logitechController.b().onTrue(new InstantCommand(() -> {

//             elevatorSubsystem.setTarget(Constants.ElevatorConstants.L2Setpoint);
//             grabberSubsystem.setTarget(Constants.GrabberConstants.L2Setpoint);
//         }));
//         //l3
//         m_logitechController.a().onTrue(new InstantCommand(() -> {

//             elevatorSubsystem.setTarget(Constants.ElevatorConstants.L3Setpoint);
//             grabberSubsystem.setTarget(Constants.GrabberConstants.L3Setpoint);
//           }));
//         //loading station
//         m_logitechController.x().onTrue(new InstantCommand(() -> {

//             elevatorSubsystem.setTarget(Constants.ElevatorConstants.intakeSetpoint);
//             grabberSubsystem.setTarget(Constants.GrabberConstants.intakeSetpoint);
//           }));

//         m_logitechController.button(7).onTrue(new InstantCommand(() -> {
//             elevatorSubsystem.setTarget(Constants.ElevatorConstants.downSetpoint);
//             grabberSubsystem.setTarget(Constants.GrabberConstants.downSetpoint);
//           }));

//           m_logitechController.povDown().onTrue(new InstantCommand(() -> {
//             elevatorSubsystem.setTarget(Constants.ElevatorConstants.Algae1Setpoint);
//             grabberSubsystem.setTarget(Constants.GrabberConstants.Algae1Setpoint);
//           }));

//         m_logitechController.povUp().onTrue(new InstantCommand(() -> {
//             elevatorSubsystem.setTarget(Constants.ElevatorConstants.Algae2Setpoint);
//             grabberSubsystem.setTarget(Constants.GrabberConstants.Algae2Setpoint);
//           }));

//           m_logitechController.leftTrigger().onTrue(new InstantCommand(() -> {
//             grabberSubsystem.rotateGrabber();
//           }));
//           m_logitechController.rightTrigger().onTrue(new InstantCommand(() -> {
//             grabberSubsystem.rotateGrabberB();
//           }));

//           m_logitechController.leftBumper().toggleOnTrue(new InstantCommand(()->{grabberSubsystem.intake();}));
//         m_logitechController.leftBumper().toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
    
//         m_logitechController.rightBumper().toggleOnTrue(new InstantCommand(()->{grabberSubsystem.outtake();}));
//         m_logitechController.rightBumper().toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();})); */

//         //LOGAN

//      /*    m_xboxController.povDown().whileTrue(driveSubsystem.driveToReefPosition(0));
//     m_xboxController.povUp().whileTrue(driveSubsystem.driveToReefPosition(3));
//     m_xboxController.povUpLeft().whileTrue(driveSubsystem.driveToReefPosition(4));
//     m_xboxController.povUpRight().whileTrue(driveSubsystem.driveToReefPosition(2));
//     m_xboxController.povDownLeft().whileTrue(driveSubsystem.driveToReefPosition(5));
//     m_xboxController.povDownRight().whileTrue(driveSubsystem.driveToReefPosition(1)); 

//     m_xboxController.b().whileTrue(driveSubsystem.driveToRightIntake());
//     m_xboxController.x().whileTrue(driveSubsystem.driveToLeftIntake()); */

//   /*   m_xboxController.leftTrigger().toggleOnTrue(new InstantCommand(()->{grabberSubsystem.intake();}));
//     m_xboxController.leftTrigger().toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();}));
    
//     m_xboxController.rightTrigger().toggleOnTrue(new InstantCommand(()->{grabberSubsystem.outtake();}));
//     m_xboxController.rightTrigger().toggleOnFalse(new InstantCommand(()->{grabberSubsystem.stop();})); */




//   /*   m_xboxController.leftStick().onTrue((reefTargeter.setBranchSide(ReefBranchSide.LEFT)
//     .andThen(Commands.runOnce(() -> driveSubsystem.getDrive().field.getObject(
//                                                  "target")
//                                                                     .setPose(
//                                                                         reefTargeter.getCoralTargetPose())))));

//     m_xboxController.rightStick().onTrue(reefTargeter.setBranchSide(ReefBranchSide.RIGHT)
//     .andThen(Commands.runOnce(() -> driveSubsystem.getDrive().field.getObject(
//                                                  "target")
//                                                                     .setPose(
//                                                                         reefTargeter.getCoralTargetPose()))));

//     m_logitechController.button(8).toggleOnTrue(driveSubsystem.switchFieldRel());
//     }
    
// } */
