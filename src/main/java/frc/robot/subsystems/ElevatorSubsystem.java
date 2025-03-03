// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

  SparkMax elevatorMotor;
  SparkMax otherMotor;
  SparkClosedLoopController pidController;
  double target;
  RelativeEncoder encoder;

  public ElevatorSubsystem() {
    target = Constants.ElevatorConstants.downSetpoint;
    SparkMaxConfig otherMotorConfig = new SparkMaxConfig();
    elevatorMotor = new SparkMax(Constants.ElevatorConstants.motor1Id, MotorType.kBrushless);
    otherMotorConfig.follow(Constants.ElevatorConstants.motor1Id);
    encoder = elevatorMotor.getEncoder();
    otherMotor = new SparkMax(Constants.ElevatorConstants.motor2Id, MotorType.kBrushless);
    otherMotor.configure(otherMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController = elevatorMotor.getClosedLoopController();
    encoder.setPosition(0);
    
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


  @Override
  public void periodic(){
    moveToSetpoint();
    SmartDashboard.putNumber("Coral/Elevator/Target Position", target);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", encoder.getPosition());
  }

}
