// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.Cameras;

public class DriveToApriltag extends Command {
DriveSubsystem swerve;
Cameras m_camera;
int id;
boolean finished;

  public DriveToApriltag(DriveSubsystem dSubsystem, Cameras camera, int t_id) {
    swerve = dSubsystem;
    m_camera = camera;
    id = t_id;
    addRequirements(dSubsystem);
  }

  public void initialize() {
    swerve.aimAtTarget(m_camera);
    swerve.driveTowardTarget(id);
    finished = true;
  }

  public void execute() {

  }

  public void end(boolean interrupted) {

  }

  public boolean isFinished() {
    return finished;
  }
}
