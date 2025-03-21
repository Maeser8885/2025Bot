// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class IntakeCoral extends Command {
  ElevatorSubsystem m_elevatorSubsystem;
  GrabberSubsystem m_grabberSubsystem;
  Timer timer = new Timer();
  public IntakeCoral(ElevatorSubsystem eSubsystem, GrabberSubsystem gSubsystem) {
    m_elevatorSubsystem = eSubsystem;
    m_grabberSubsystem = gSubsystem;
    addRequirements(eSubsystem,gSubsystem);
  }

  @Override
  public void initialize(){
    timer.restart();
  }
  

  @Override
  public void execute(){
    m_elevatorSubsystem.setTarget(Constants.ElevatorConstants.intakeSetpoint);
    m_grabberSubsystem.setTarget(Constants.GrabberConstants.intakeSetpoint);
    m_grabberSubsystem.rotateGrabberB();

    if(timer.get() >= 0.6){
    m_grabberSubsystem.outtake();}

    if(timer.get() >= 0.99){
      m_grabberSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished(){
    return timer.get() >= 1.0;
  }
}
