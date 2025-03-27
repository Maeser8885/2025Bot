// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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
  double m_speed = 1;
  public RelativeEncoder encoder;

  boolean goUp = false;
  boolean goDown = false;

  public ElevatorSubsystem() {
    // initialize the elevator target
    target = Constants.ElevatorConstants.downSetpoint;
    // make configs for motors
    SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
    // add softlimits
    SoftLimitConfig softlimits = new SoftLimitConfig();
    softlimits.forwardSoftLimit(Constants.ElevatorConstants.downSetpoint);
    softlimits.reverseSoftLimit(Constants.ElevatorConstants.upSoftLimit);
    // apply softlimits to configs
    elevatorMotorConfig.apply(softlimits);
    elevatorMotorConfig.closedLoop.p(0.01).i(0).d(0);
    // make motors
    elevatorMotor = new SparkMax(Constants.ElevatorConstants.motorId, MotorType.kBrushless);
    // set follwer
    // get encoder
    encoder = elevatorMotor.getEncoder();
    // config motors
    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController = elevatorMotor.getClosedLoopController();
  }

  public void moveToSetpoint() {
    pidController.setReference(target, ControlType.kPosition);
  }

  public void setTarget(double setpoint) {
    target = setpoint;
  }

  public void moveUp() {
    goUp = true;
  }

  public void moveDown() {
    goDown = true;
  }

  public void move(double speed){
    m_speed = -speed;
    goUp = true;
  }

  public void stop() {
    goUp = false;
    goDown = false;
  }

  public Command elevatorJoystickMoveCommand(){
    return new RunCommand(() -> move(RobotContainer.m_xboxController.getLeftY()), this);
  }

  public Command logitechElevatorCommand(){
    return new RunCommand(() -> move(RobotContainer.m_logitechController.getLeftY()), this);
  }

  @Override
  public void periodic() {
    if (goUp && target < 240) {
      target += 0.4 * m_speed;
    }
    if (goDown && target > 1) {
      target -= 0.4 * m_speed;
    }
    moveToSetpoint();
    SmartDashboard.putNumber("Elevator target", target);
    SmartDashboard.putNumber("Elevator position", elevatorMotor.getEncoder().getPosition());
  }

}
