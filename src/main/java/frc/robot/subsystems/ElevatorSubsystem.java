// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

  SparkMax elevatorMotor;
  SparkClosedLoopController pidController;

  public ElevatorSubsystem() {
    elevatorMotor = new SparkMax(Constants.ElevatorConstants.motorId, MotorType.kBrushless);
    pidController = elevatorMotor.getClosedLoopController();
  }

  
  



  
}
