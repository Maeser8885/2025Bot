// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746),
        Units.inchesToMeters(158.501));
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

    public static final ArrayList<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>(13); // Starting at the
                                                                                                  // right branch facing
                                                                                                  // the driver station
                                                                                                  // in clockwise

    static {
      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }
        branchPositions.add(fillLeft);
        branchPositions.add(fillRight);
      }
    }

    public enum ReefHeight {
      L4(Units.inchesToMeters(54), 44),
      L3(Units.inchesToMeters(48), 14),
      L2(Units.inchesToMeters(42), -14),
      L1(Units.inchesToMeters(42), -56);

      public final double height;
      public final double pitch;

      ReefHeight(double height, double pitch) {
        this.height = height;
        this.pitch = pitch; // in degrees
      }
    }

    public static Pose2d middleOfCages = new Pose2d(Units.inchesToMeters(345.428), Units.inchesToMeters(230),
        Rotation2d.fromDegrees(0));
  }

  public static class AutoScoring {
    public static class Processor {
      public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
          Inches.of(0).in(Meters),
          Rotation2d.fromDegrees(0));
    }

    public static class Reef {
      // x + front ->, y + left
      public static final Transform2d coralOffset = new Transform2d(Inches.of(29).in(Meters),
          Inches.of(5).in(Meters),
          Rotation2d.fromDegrees(180));
      public static final Transform2d algaeOffset = new Transform2d(Inches.of(23).in(Meters),
          Inches.of(-19).in(Meters),
          Rotation2d.fromDegrees(180));
    }

    public static class HumanPlayer {

      public static class Left {

        public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
            Inches.of(0).in(Meters),
            Rotation2d.fromDegrees(0));
      }

      public static class Right {

        public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
            Inches.of(0).in(Meters),
            Rotation2d.fromDegrees(0));
      }
    }
  }
}
