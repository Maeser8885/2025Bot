// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants{
    //feet/sec
    public static final double maxSpeed = 20;
  }

  public static class VisionConstants{
    public static final String cameraName = "centerCam";
  }

  public static class ElevatorConstants{
    public static final int motorId = 9;
    public static final double downSetpoint = 0;
    public static final double intakeSetpoint = 70;
    public static final double L1Setpoint = 50;
    public static final double L2Setpoint = 90;
    public static final double L3Setpoint = 150;
    public static final double L4Setpoint = 240;
    public static final double upSoftLimit = 242;
  }

  public static class GrabberConstants{
    public static final int rotationMotorId = 13;
    public static final int opencloseMotorId = 12;
    public static final int sidewaysMotorId = 11;
    public static final double downSetpoint = 0;
    public static final double intakeSetpoint = -5.8;
    public static final double L1Setpoint = -7;
    public static final double L2Setpoint = -6.929;
    public static final double L3Setpoint = -6.929;
    public static final double L4Setpoint = -3.4;
    public static double upSoftLimit = 0.214;
    public static final double backSoftLimit = -14;
    public static final double openclosespeed = 1.0;
    
    public static final double rotationTime = 0.1;
    public static final double rotationSpeed = 0.3;
  }
}
