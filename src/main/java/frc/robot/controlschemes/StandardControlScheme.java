package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

public class StandardControlScheme extends ControlScheme{

    String name = "Standard Controls";

     public StandardControlScheme() {
        super();
    }

    public void configureBindings(){
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
    m_driverController.button(7).onTrue(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.intakeSetpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.intakeSetpoint);
    }));
    //8 = L4
    m_driverController.button(8).onTrue(new InstantCommand(() -> {

      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L4Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L4Setpoint);
    }));
    //9 = L2
    m_driverController.button(9).onTrue(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L2Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L2Setpoint);
    }));
    //10 = L3
    m_driverController.button(10).onTrue(new InstantCommand(() -> {
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L3Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L3Setpoint);
    }));
    //11 = DOWN
    m_driverController.button(11).onTrue(
      new InstantCommand(() -> {
        elevatorSubsystem.setTarget(Constants.ElevatorConstants.downSetpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.downSetpoint);
    }));
    //12 = L1
    m_driverController.button(12).onTrue(
      new InstantCommand(() -> {
       
        elevatorSubsystem.setTarget(Constants.ElevatorConstants.L1Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L1Setpoint);})
      );
        //rotate grabber
    m_xboxController.leftTrigger().onTrue(new InstantCommand(()->{grabberSubsystem.rotateGrabber();}));
    m_xboxController.rightTrigger().onTrue(new InstantCommand(()->{grabberSubsystem.rotateGrabberB();}));

    m_xboxController.leftTrigger().onFalse(new InstantCommand(()->{grabberSubsystem.stopGrabber();}));
    m_xboxController.rightTrigger().onFalse(new InstantCommand(()->{grabberSubsystem.stopGrabber();}));

    //3 = change field relativity
    m_driverController.button(3).toggleOnTrue(driveSubsystem.switchFieldRel());
          //manual elbow rotation with xbox controller bumpers
    m_xboxController.povUp().onTrue(new InstantCommand(() -> {grabberSubsystem.rotateElbowB();}));
    m_xboxController.povDown().onTrue(new InstantCommand(() -> {grabberSubsystem.rotateElbow();}));

    m_xboxController.povUp().onFalse(new InstantCommand(() -> {grabberSubsystem.stopElbow();}));
    m_xboxController.povDown().onFalse(new InstantCommand(() -> {grabberSubsystem.stopElbow();}));
    //manual elevator movement with xbox controller triggers
    m_xboxController.rightStick().onTrue(new InstantCommand(()->{elevatorSubsystem.moveDown();}));
    m_xboxController.leftStick().onTrue(new InstantCommand(()->{elevatorSubsystem.moveUp();}));

    m_xboxController.rightStick().onFalse(new InstantCommand(()-> {elevatorSubsystem.stop();}));
    m_xboxController.leftStick().onFalse(new InstantCommand(()-> {elevatorSubsystem.stop();}));

        //xbox presets - y:L1 b:L2 a:L3 x:L4 dpadl:down dpadr:up 
    m_xboxController.y().onTrue(new InstantCommand(() -> {
       
      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L1Setpoint);
    grabberSubsystem.setTarget(Constants.GrabberConstants.L1Setpoint);}));

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

      elevatorSubsystem.setTarget(Constants.ElevatorConstants.L4Setpoint);
      grabberSubsystem.setTarget(Constants.GrabberConstants.L4Setpoint);
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
}
