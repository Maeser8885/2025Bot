// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

public class GrabberSubsystem extends SubsystemBase {

  SparkMax grabberMotor;
  SparkMax releaseMotor;
  SparkClosedLoopController pidController;
  double target;
  RelativeEncoder encoder;

  public GrabberSubsystem() {
    //initialize the grabber target
    target = Constants.GrabberConstants.downSetpoint;
    //make configs for motor
    SparkMaxConfig grabberMotorConfig = new SparkMaxConfig();
    //add softlimits
    SoftLimitConfig softlimits = new SoftLimitConfig();
    softlimits.forwardSoftLimit(Constants.ElevatorConstants.upSoftLimit);
    softlimits.reverseSoftLimit(Constants.GrabberConstants.backSoftLimit);
    //apply softlimits to config
    grabberMotorConfig.apply(softlimits);
    //make motors
    grabberMotor = new SparkMax(Constants.GrabberConstants.rotationMotorId, MotorType.kBrushless);
    //get encoder
    encoder = grabberMotor.getEncoder();
    //config motors
    grabberMotor.configure(grabberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    pidController = grabberMotor.getClosedLoopController();
    encoder.setPosition(0);

    releaseMotor = new SparkMax(Constants.GrabberConstants.opencloseMotorId, MotorType.kBrushless);
  }

   public void moveToSetpoint() {
    pidController.setReference(target, ControlType.kMAXMotionPositionControl);
  }

    public Command setTargetCommand(double setpoint) {
    return this.runOnce(
        () -> {
          target = setpoint;
        });
  }

  public Command getIntakeCommand(){
    return this.run(() -> {
      releaseMotor.set(Constants.GrabberConstants.openclosespeed);
    });
  }

  public Command getOuttakeCommand(){
    return this.run(() -> {
      releaseMotor.set(-Constants.GrabberConstants.openclosespeed);
    });
  }

  public Command getStopCommand(){
    return this.run(() -> {
      releaseMotor.set(0);
    });
  }

  @Override
  public void periodic(){
    moveToSetpoint();
    SmartDashboard.putNumber("Grabber/Target Position", target);
    SmartDashboard.putNumber("Grabber/Actual Position", encoder.getPosition());
  }

}
