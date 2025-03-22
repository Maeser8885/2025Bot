package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoScoring;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class ReefTarget {

  private ReefBranch targetBranch;
  private ReefBranchLevel targetBranchLevel;
  private ReefBranchSide targetReefBranchSide = ReefBranchSide.CLOSEST;

  private List<Pose2d> reefBranches = null;
  private List<Pose2d> allianceRelativeReefBranches = null;
  private Map<Pose2d, ReefBranch> reefPoseToBranchMap = null;

  private void initializeBranchPoses() {
    reefBranches = new ArrayList<>();
    reefPoseToBranchMap = new HashMap<>();
    for (int branchPositionIndex = 0; branchPositionIndex < Constants.FieldConstants.branchPositions.size(); branchPositionIndex++) {
      Map<ReefHeight, Pose3d> branchPosition = Constants.FieldConstants.branchPositions.get(branchPositionIndex);
      Pose2d targetPose = branchPosition.get(ReefHeight.L4).toPose2d();
      reefBranches.add(targetPose);
      reefPoseToBranchMap.put(targetPose, ReefBranch.values()[branchPositionIndex]);
      reefPoseToBranchMap.put(PoseAlliances.flip(targetPose), ReefBranch.values()[branchPositionIndex]);
    }
    allianceRelativeReefBranches = reefBranches.stream().map(PoseAlliances::apply).collect(Collectors.toList());
  }

  private static int rightBranchOrdinal(ReefBranch branch) {
    boolean isRight = (branch.ordinal() + 1) % 2 == 0;
    return MathUtil.clamp(branch.ordinal() + (isRight ? 0 : 1), 0, 11);
  }

  private static int leftBranchOrdinal(ReefBranch branch) {
    boolean isRight = (branch.ordinal() + 1) % 2 == 0;
    return MathUtil.clamp(branch.ordinal() - (isRight ? 1 : 0), 0, 11);
  }

  public int getTargetBranchOrdinal() {
    if (targetBranch != null) {
      switch (targetReefBranchSide) {
        case CLOSEST -> {
          return targetBranch.ordinal();
        }
        case RIGHT -> {
          return rightBranchOrdinal(targetBranch);
        }
        case LEFT -> {
          return leftBranchOrdinal(targetBranch);
        }
      }
    }
    return 0;
  }

  public ReefTarget() {
    RobotModeTriggers.autonomous().onFalse(Commands.runOnce(this::initializeBranchPoses));
  }

  public void setTarget(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel) {
    this.targetBranch = targetBranch;
    this.targetBranchLevel = targetBranchLevel;
  }

  public Command setTargetCommand(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel) {
    return Commands.runOnce(() -> setTarget(targetBranch, targetBranchLevel));
  }

  public Command setBranchCommand(ReefBranch branch) {
    return Commands.runOnce(() -> {
      targetBranch = branch;
    });
  }

  public Command setBranchSide(ReefBranchSide side) {
    return Commands.runOnce(() -> {
      targetReefBranchSide = side;
    });
  }

  public Command setBranchLevel(ReefBranchLevel level) {
    return Commands.runOnce(() -> {
      targetBranchLevel = level;
    });
  }

  public ReefBranchLevel getTargetBranchLevel() {
    return targetBranchLevel;
  }

  public ReefBranch getTargetBranch() {
    return targetBranch;
  }

  public Command driveToCoralTarget(DriveSubsystem swerveDrive) {
    return (Commands.runOnce(() -> {
      swerveDrive.getDrive().field.getObject("target").setPose(getCoralTargetPose());
    })).andThen(swerveDrive.driveToPose(getCoralTargetPose()));
  }

  public Command driveToAlgaeTarget(DriveSubsystem swerveDrive) {
    return (Commands.runOnce(() -> {
      swerveDrive.getDrive().field.getObject("target").setPose(getAlgaeTargetPose());
    })).andThen(swerveDrive.driveToPose(getAlgaeTargetPose()));
  }

  public Pose2d getCoralTargetPose() {
    Pose2d scoringPose = Pose2d.kZero;
    if (targetBranch != null) {
      Pose2d startingPose = PoseAlliances.apply(Constants.FieldConstants.branchPositions.get(getTargetBranchOrdinal()).get(ReefHeight.L2)
          .toPose2d());
      scoringPose = startingPose.plus(AutoScoring.Reef.coralOffset);
      SmartDashboard.putString("Targetted Coral Pose with Offset (Meters)", scoringPose.toString());

    }
    return scoringPose;
  }

  public Pose2d getAlgaeTargetPose() {
    Pose2d scoringPose = Pose2d.kZero;
    if (targetBranch != null) {
      Pose2d startingPose = PoseAlliances.apply(Constants.FieldConstants.branchPositions.get(rightBranchOrdinal(targetBranch))
          .get(ReefHeight.L2).toPose2d());
      scoringPose = startingPose.plus(AutoScoring.Reef.algaeOffset);

    }
    return scoringPose;
  }

  public Pose2d autoTarget(Supplier<Pose2d> currentPose) {
    if (reefBranches == null) {
      initializeBranchPoses();
    }

    Pose2d selectedTargetPose = currentPose.get().nearest(allianceRelativeReefBranches);
    targetBranch = reefPoseToBranchMap.get(selectedTargetPose);
    return selectedTargetPose;
  }

  public Command autoTargetCommand(Supplier<Pose2d> currentPose) {
    return Commands.runOnce(() -> autoTarget(currentPose));
  }

  public void increaseBranch() {
    if (targetBranch != null) {
      targetBranch = ReefBranch.values()[(targetBranch.ordinal() + 1) % 12];
    }
  }

  public void printTargetPose(ReefBranchLevel level, ReefBranchSide side) {

    targetBranchLevel = level;
    targetReefBranchSide = side;

    if (targetBranch != null)
      System.out
          .println("Coral Branch: " + targetBranch.toString() + " Target Pose: " + getCoralTargetPose().toString());
  }

  public void printTargetPose(ReefBranch branch, ReefBranchLevel level, ReefBranchSide side) {
    targetBranch = branch;
    printTargetPose(level, side);
  }

  public void setCoralTargetOnField(DriveSubsystem swerveDrive) {
    swerveDrive.getDrive().field.getObject("target").setPose(getCoralTargetPose());
  }

  public void setAlgaeTargetOnField(DriveSubsystem swerveDrive) {
    swerveDrive.getDrive().field.getObject("target").setPose(getAlgaeTargetPose());
  }

  public enum ReefBranch {
    A, B, K, L, I, J, G, H, E, F, C, D
  }

  public enum ReefBranchLevel {
    L2, L3, L1, L4
  }

  public enum ReefBranchSide {
    CLOSEST, RIGHT, LEFT
  }

}