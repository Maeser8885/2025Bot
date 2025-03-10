// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class DepositCoral extends Command {
  ElevatorSubsystem m_elevatorSubsystem;
  GrabberSubsystem m_grabberSubsystem;
  int level;
  boolean finished;
  //1, 2, 3, 4 - L1, L2, L3, L4
  //0 - down
  //5 - intake
  public DepositCoral(ElevatorSubsystem eSubsystem, GrabberSubsystem gSubsystem, int rlevel) {
    m_elevatorSubsystem = eSubsystem;
    m_grabberSubsystem = gSubsystem;
    level = rlevel;
    addRequirements(eSubsystem,gSubsystem);
    finished = false;
  }

  @Override
  public void initialize(){
    if(level == 4){
    m_elevatorSubsystem.setTargetCommand(Constants.ElevatorConstants.L4Setpoint).andThen(
    m_grabberSubsystem.setTargetCommand(Constants.GrabberConstants.L4Setpoint)).andThen(()->{finished = true;});
  }
    if(level == 3){
      m_elevatorSubsystem.setTargetCommand(Constants.ElevatorConstants.L3Setpoint).andThen(
      m_grabberSubsystem.setTargetCommand(Constants.GrabberConstants.L3Setpoint)).andThen(()->{finished = true;});
    }
    if(level == 2){
        m_elevatorSubsystem.setTargetCommand(Constants.ElevatorConstants.L2Setpoint).andThen(
        m_grabberSubsystem.setTargetCommand(Constants.GrabberConstants.L2Setpoint)).andThen(()->{finished = true;});
      }
      if(level == 1){
        m_elevatorSubsystem.setTargetCommand(Constants.ElevatorConstants.L1Setpoint).andThen(
        m_grabberSubsystem.setTargetCommand(Constants.GrabberConstants.L1Setpoint)).andThen(()->{finished = true;});
  }
  if(level == 0){
    m_elevatorSubsystem.setTargetCommand(Constants.ElevatorConstants.downSetpoint).andThen(
    m_grabberSubsystem.setTargetCommand(Constants.GrabberConstants.downSetpoint)).andThen(()->{finished = true;});
  }
  if(level == 5){
    m_elevatorSubsystem.setTargetCommand(Constants.ElevatorConstants.intakeSetpoint).andThen(
    m_grabberSubsystem.setTargetCommand(Constants.GrabberConstants.intakeSetpoint)).andThen(()->{finished = true;});
  }
  }

  @Override
  public void execute(){
    
  }

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished(){
    return finished;
  }
}
