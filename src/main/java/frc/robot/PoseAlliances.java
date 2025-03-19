package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class PoseAlliances {
    public static double applyX(double x)
  {
    return shouldFlip() ? Units.inchesToMeters(690.876) - x : x;
  }

  public static double applyY(double y)
  {
    return shouldFlip() ? Units.inchesToMeters(317) - y : y;
  }

  public static Translation2d apply(Translation2d translation)
  {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation)
  {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose)
  {
    return shouldFlip()
           ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
           : pose;
  }

  public static Pose2d flip(Pose2d pose)
  {
    return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static boolean shouldFlip()
  {
    return (DriverStation.getAlliance().isPresent()  && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
  }

  
}
