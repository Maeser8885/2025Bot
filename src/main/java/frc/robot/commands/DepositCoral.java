// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class DepositCoral extends Command {
  ElevatorSubsystem m_elevatorSubsystem;
  GrabberSubsystem m_grabberSubsystem;
  Timer timer = new Timer();
  public DepositCoral(ElevatorSubsystem eSubsystem, GrabberSubsystem gSubsystem) {
    m_elevatorSubsystem = eSubsystem;
    m_grabberSubsystem = gSubsystem;
    addRequirements(eSubsystem,gSubsystem);
  }

  @Override
  public void initialize(){
    m_elevatorSubsystem.setTarget(Constants.ElevatorConstants.L2Setpoint);
    m_grabberSubsystem.setTarget(Constants.GrabberConstants.L2Setpoint);
  }
  

  @Override
  public void execute(){
    m_grabberSubsystem.outtake();
  }

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished(){
    return timer.get() >= 0.5;
  }
}
