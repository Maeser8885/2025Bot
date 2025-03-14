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

public class GrabberSubsystem extends SubsystemBase {

  public SparkMax grabberMotor;
  SparkMax releaseMotor;
  SparkMax sidewaysMotor;
  SparkClosedLoopController pidController;
  double target;
  public RelativeEncoder encoder;
  boolean rotated;
  boolean goUp;
  boolean goDown;

  public GrabberSubsystem() {
    rotated = false;
    //initialize the grabber target
    target = -8;



    //make configs for motor
    SparkMaxConfig grabberMotorConfig = new SparkMaxConfig();
    
    //add softlimits
    SoftLimitConfig softlimits = new SoftLimitConfig();
    softlimits.forwardSoftLimit(Constants.ElevatorConstants.upSoftLimit);
    softlimits.reverseSoftLimit(Constants.GrabberConstants.backSoftLimit);
    //apply softlimits to config
    grabberMotorConfig.apply(softlimits);
    grabberMotorConfig.closedLoop.p(0.05).i(0).d(0.8);
    //make motors
    grabberMotor = new SparkMax(Constants.GrabberConstants.rotationMotorId, MotorType.kBrushless);
    sidewaysMotor = new SparkMax(Constants.GrabberConstants.sidewaysMotorId, MotorType.kBrushless);
    //get encoder
    encoder = grabberMotor.getEncoder();
    //config motors
    grabberMotor.configure(grabberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    pidController = grabberMotor.getClosedLoopController();
    encoder.setPosition(0);

    releaseMotor = new SparkMax(Constants.GrabberConstants.opencloseMotorId, MotorType.kBrushless);
  }

   public void moveToSetpoint() {
    pidController.setReference(target, ControlType.kPosition);
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

  public void fastOuttake(){
    releaseMotor.set(1);
  }

  public void stop(){
      releaseMotor.set(0);
  }

  public void stopGrabber(){
    sidewaysMotor.set(0);
  }

  public void rotateGrabber(){
    if(encoder.getPosition() < -0.1){
      sidewaysMotor.set(-0.1);
    }
    else{
      System.out.println("Grabber Not In Position");
    }
  }

  public void rotateGrabberB(){
    if(encoder.getPosition() < -0.1){
      sidewaysMotor.set(0.1);
    }
    else{
      System.out.println("Grabber Not In Position");
    }
  }

  public void rotateElbow(){
    goUp = true;
    goDown = false;
  }

  public void rotateElbowB(){
    goDown = true;
    goUp = false;
  }

  public void stopElbow(){
    goUp = false;
    goDown = false;
  }

  @Override
  public void periodic(){
    moveToSetpoint();
    if(goUp && target < -1){
      target -= 0.1;
    }
    if(goDown && target > -9){
      target += 0.1;
    }
    
    SmartDashboard.putNumber("Grabber target", target);
    SmartDashboard.putNumber("Grabber position", encoder.getPosition());

  }

}
