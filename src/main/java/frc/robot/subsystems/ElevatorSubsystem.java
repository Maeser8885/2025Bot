// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

  SparkMax elevatorMotor;
  SparkClosedLoopController pidController;
  double target;
  RelativeEncoder encoder;

  public ElevatorSubsystem() {
    //initialize the elevator target
    target = Constants.ElevatorConstants.downSetpoint;
    //make configs for motors
    SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
    //add softlimits
    SoftLimitConfig softlimits = new SoftLimitConfig();
    softlimits.forwardSoftLimit(Constants.ElevatorConstants.downSetpoint);
    softlimits.reverseSoftLimit(Constants.ElevatorConstants.upSoftLimit);
    //apply softlimits to configs
    elevatorMotorConfig.apply(softlimits);
    elevatorMotorConfig.closedLoop.p(0.01).i(0).d(0);
    //make motors
    elevatorMotor = new SparkMax(Constants.ElevatorConstants.motorId, MotorType.kBrushless);
    //set follwer
    //get encoder
    encoder = elevatorMotor.getEncoder();
    //config motors
    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    pidController = elevatorMotor.getClosedLoopController();
    encoder.setPosition(0);
  }

   public void moveToSetpoint() {
    pidController.setReference(target, ControlType.kPosition);
  }

    public void setTarget(double setpoint) {
        target = setpoint;
  }


  @Override
  public void periodic(){
    moveToSetpoint();
    SmartDashboard.putNumber("Elevator target", target);
    SmartDashboard.putNumber("Elevator position", elevatorMotor.getEncoder().getPosition());
  }

}
