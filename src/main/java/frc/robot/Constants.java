// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    // feet/sec
    public static final double maxSpeed = 20;
  }

  public static class VisionConstants {
    public static final String cameraName = "centerCam";
  }

  public static class ElevatorConstants {
    public static final int motorId = 9;
    public static final double downSetpoint = 0;
    public static final double autoSetpoint = 60;
    public static final double intakeSetpoint = 70;
    public static final double L1Setpoint = 50;
    public static final double L2Setpoint = 41.73;
    public static final double L3Setpoint = 146.8;
    public static final double L4Setpoint = 240;
    // public static final double Algae1Setpoint = 127.6;
    // public static final double Algae2Setpoint = 193.4;
    public static final double upSoftLimit = 242;
  }

  public static class GrabberConstants {
    public static final int rotationMotorId = 13;
    public static final int opencloseMotorId = 12;
    public static final int sidewaysMotorId = 11;
    public static final double downSetpoint = -3;
    public static final double autoSetPoint = -5.8;
    public static final double intakeSetpoint = -5.8;
    public static final double L1Setpoint = -7;
    public static final double L2Setpoint = -1.73;
    public static final double L3Setpoint = -3.7;
    public static final double L4Setpoint = -3.4;
    // public static final double Algae1Setpoint = -7.6;
    // public static final double Algae2Setpoint = -7.6;
    public static double upSoftLimit = 0.214;
    public static final double backSoftLimit = -14;
    public static final double openclosespeed = 0.6;
    public static final double pSetpoint = -7;
  }

  public static class FieldConstants {
    public static final Pose2d leftIntake = new Pose2d(
        Units.inchesToMeters(33.526),
        Units.inchesToMeters(25.824),
        Rotation2d.fromDegrees(54.011));

    public static final Pose2d rightIntake = new Pose2d(
        leftIntake.getX(),
        Units.inchesToMeters(317) - leftIntake.getY(),
        Rotation2d.fromRadians(-leftIntake.getRotation().getRadians()));

    public static final Pose2d Processer = new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));

    public static final Pose2d[] reefPositions = new Pose2d[] {
        new Pose2d(
            Units.inchesToMeters(144.003),
            Units.inchesToMeters(158.500),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            Units.inchesToMeters(160.373),
            Units.inchesToMeters(186.857),
            Rotation2d.fromDegrees(120)),
        new Pose2d(
            Units.inchesToMeters(193.116),
            Units.inchesToMeters(186.858),
            Rotation2d.fromDegrees(60)),
        new Pose2d(
            Units.inchesToMeters(209.489),
            Units.inchesToMeters(158.502),
            Rotation2d.fromDegrees(0)),
        new Pose2d(
            Units.inchesToMeters(193.118),
            Units.inchesToMeters(130.145),
            Rotation2d.fromDegrees(-60)),
        new Pose2d(
            Units.inchesToMeters(160.375),
            Units.inchesToMeters(130.144),
            Rotation2d.fromDegrees(-120))
    }; // Starting from driver station going in clockwise order

    public static Pose2d middleOfCages = new Pose2d(Units.inchesToMeters(345.428), Units.inchesToMeters(230), Rotation2d.fromDegrees(0));
  }
}
