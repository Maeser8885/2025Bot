// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends Command {

  DriveSubsystem dSub;
  private final double m_speed;
  private final double m_time;
  private final double m_rotation;
  Timer timer = new Timer();
  public DriveDistance(DriveSubsystem driveSub, double speed, double time, double rotation) {
    dSub = driveSub;
    m_speed = speed;
    m_time = time;
    m_rotation = rotation;
    addRequirements(driveSub);
  }

  @Override
  public void initialize(){
    timer.reset();
    timer.start();
  }
  
  @Override
  public void execute(){
    dSub.drive(m_speed, 0, m_rotation, false, 1);
  }

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished(){
    return timer.get() > m_time;
  }
}
