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
    public static final double maxSpeed = 13.8940937368;
    public static boolean visionEnabled = true;
  }

  public static class VisionConstants{
    public static final String cameraName = "centerCam";
  }

  public static class ElevatorConstants{
    public static final int motor1Id = 9;
    public static final int motor2Id = 10;
    public static final double downSetpoint = 0;
    public static final double intakeSetpoint = 0;
    public static final double L1Setpoint = 0;
    public static final double L2Setpoint = 0;
    public static final double L3Setpoint = 0;
    public static final double L4Setpoint = 0;
    public static final double upSoftLimit = 0;
  }

  public static class GrabberConstants{
    public static final int rotationMotorId = 11;
    public static final int opencloseMotorId = 12;
    public static final int sidewaysMotorId = 13;
    public static final double downSetpoint = 0;
    public static final double intakeSetpoint = 0;
    public static final double L1Setpoint = 0;
    public static final double L2Setpoint = 0;
    public static final double L3Setpoint = 0;
    public static final double L4Setpoint = 0;
    public static double upSoftLimit = 0;
    public static final double backSoftLimit = 0;
    public static final double openclosespeed = 1.0;
    
    public static final double rotationTime = 1.0;
    public static final double rotationSpeed = 1.0;
  }
}
