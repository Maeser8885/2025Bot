// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

  SparkMax motor1;
  SparkMax motor2;

  public ElevatorSubsystem() {
    motor1 = new SparkMax(Constants.ElevatorConstants.motor1, MotorType.kBrushless);
    motor2 = new SparkMax(Constants.ElevatorConstants.motor2, MotorType.kBrushless);
  }

  public void go(){
    motor1.set(0.5);
    motor2.set(-0.5);
  }

  public void unGo(){
    motor1.set(-0.5);
    motor2.set(0.5);
  }

  
}
