// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
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

public class GrabberSubsystem extends SubsystemBase {

  public SparkMax grabberMotor;
  SparkMax releaseMotor;
  SparkMax sidewaysMotor;
  SparkClosedLoopController pidController;
  SparkClosedLoopController rPidController;
  double target;
  double rTarget;
  RelativeEncoder rEncoder;
  RelativeEncoder encoder;
  Timer rotationTimer = new Timer();
  boolean rotated;

  public GrabberSubsystem() {
    rotated = false;
    //initialize the grabber target
    target = -8;
    rTarget = 0;

    //make configs for motor
    SparkMaxConfig grabberMotorConfig = new SparkMaxConfig();
    
    //add softlimits
    SoftLimitConfig softlimits = new SoftLimitConfig();
    SoftLimitConfig rSoftLimits = new SoftLimitConfig();
    SparkMaxConfig rMotorConfig = new SparkMaxConfig();
    rMotorConfig.closedLoop.p(0.05).i(0).d(0.8);
    softlimits.forwardSoftLimit(Constants.ElevatorConstants.upSoftLimit);
    softlimits.reverseSoftLimit(Constants.GrabberConstants.backSoftLimit);
    rSoftLimits.forwardSoftLimit(0);
    rSoftLimits.reverseSoftLimit(5.071);
    rSoftLimits.forwardSoftLimitEnabled(true);
    rSoftLimits.reverseSoftLimitEnabled(true);
    rMotorConfig.apply(rSoftLimits);
    //apply softlimits to config
    grabberMotorConfig.apply(softlimits);
    grabberMotorConfig.closedLoop.p(0.05).i(0).d(0.8);
    //make motors
    grabberMotor = new SparkMax(Constants.GrabberConstants.rotationMotorId, MotorType.kBrushless);
    sidewaysMotor = new SparkMax(Constants.GrabberConstants.sidewaysMotorId, MotorType.kBrushless);
    //get encoder
    encoder = grabberMotor.getEncoder();
    rEncoder = sidewaysMotor.getEncoder();
    //config motors
    grabberMotor.configure(grabberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sidewaysMotor.configure(rMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    pidController = grabberMotor.getClosedLoopController();
    rPidController = sidewaysMotor.getClosedLoopController();
    encoder.setPosition(0);

    releaseMotor = new SparkMax(Constants.GrabberConstants.opencloseMotorId, MotorType.kBrushless);
    rEncoder.setPosition(0);
  }

   public void moveToSetpoint() {
    pidController.setReference(target, ControlType.kPosition);
    rPidController.setReference(rTarget, ControlType.kPosition);
  }

    public void setTarget(double setpoint){
          target = setpoint;
  }

  public void intake(){
    releaseMotor.set(Constants.GrabberConstants.openclosespeed);
  }

  public void outtake(){
    releaseMotor.set(-Constants.GrabberConstants.openclosespeed);
  }

  public void stop(){
      releaseMotor.set(0);
  }

  public void rotateGrabber(){
    if(rotated){rTarget = 0; rotated = false;}
    else{rTarget = 5.071; rotated = true;}
  }

  @Override
  public void periodic(){
    moveToSetpoint();
    SmartDashboard.putNumber("Grabber target", target);
    SmartDashboard.putNumber("Grabber position", encoder.getPosition());
    SmartDashboard.putNumber("Sideways target", rTarget);
    SmartDashboard.putNumber("Sideways position", rEncoder.getPosition());

  }

}
