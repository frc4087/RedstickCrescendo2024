// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveWheelModuleSubsystem;
import frc. robot.Pigeon2Handler;
//import frc.robot.constants.CompConstants;
import frc.robot.VisionConstants;



public class PoseEstimation extends SubsystemBase {
  private final PhotonCameras mCameras;
  private final SwerveDriveSubsystem swerveDrive;
  private final Pigeon2Handler pigeon;
  private final SwerveWheelModuleSubsystem swerveWheelModuleSubsystem;
  private final SwerveDrivePoseEstimator mPoseEstimator;
  private final Field2d mField = new Field2d();


  public PoseEstimation(PhotonCameras lime, SwerveDriveSubsystem swerve) {
    mCameras = lime;
    swerveDrive = swerve;

    mPoseEstimator = new SwerveDrivePoseEstimator(swerveWheelModuleSubsystem.getKinematics(),
        swerveDrive.getRobotAngle(), swerveWheelModuleSubsystem.getPosition(), swerveDrive.getRobotPose(),
        VisionConstants.STATE_STD_DEVS, VisionConstants.VISION_MEASUREMENTS_STD_DEVS);
    SmartDashboard.putData("Field", mField);
  }

  @Override
  public void periodic() {
    mPoseEstimator.update(pigeon.getAngleDeg(), swerveWheelModuleSubsystem.getSwerveModulePosition());
    updateVision();

    Pose2d pose = mPoseEstimator.getEstimatedPosition();
    mField.setRobotPose(pose);
    SmartDashboard.putData("Field", mField);

      SmartDashboard.putNumber("PoseEst X", pose.getX());
      SmartDashboard.putNumber("PoseEst Y", pose.getY());
      SmartDashboard.putNumber("PoseEst Rot", pose.getRotation().getDegrees());

  }

  public void updateVision() {
    boolean updatedFront = updateVisionFront();
    //boolean updatedBack = updateVisionFront();

    // if (CompConstants.DEBUG_MODE) {
    //   SmartDashboard.putBoolean("Updating Front", updatedFront);
    //   SmartDashboard.putBoolean("Updating Back", updatedBack);
    // }
  }

  // Get Pose From Back Camera
//   public boolean updateVisionBack() {
//     Optional<EstimatedRobotPose> pose =
//         mCameras.getEstimatedGlobalPoseBack(mPoseEstimator.getEstimatedPosition());
//     if (pose.isPresent()) {
//       EstimatedRobotPose camPose = pose.get();
//       if (isValidPose(camPose)) {
//         mPoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
//             camPose.timestampSeconds);
//         return true;
//       }
//     }
//     return false;
//   }

  // Get Pose From Front Camera
  public boolean updateVisionFront() {
    Optional<EstimatedRobotPose> pose =
        mCameras.getEstimatedGlobalPoseFront(mPoseEstimator.getEstimatedPosition());
    if (pose.isPresent()) {
      EstimatedRobotPose camPose = pose.get();
      if (isValidPose(camPose)) {
        mPoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
            camPose.timestampSeconds);
        return true;
      }
    }

    return false;
  }

  public boolean isValidPose(EstimatedRobotPose pose) {
    List<PhotonTrackedTarget> targets = pose.targetsUsed;
    if (targets.size() == 1) {
      return targets.get(0).getPoseAmbiguity() < VisionConstants.SINGLE_TAG_AMBIGUITY_THRESH;
    }

    return true;
  }

  public Pose2d getRobotPose() {
    return mPoseEstimator.getEstimatedPosition();

  }

  public void resetPose() {
    mPoseEstimator.resetPosition(swerveDrive.getRobotAngle(), swerveWheelModuleSubsystem.getSwerveModulePosition(),
        new Pose2d());
  }

  public void resetPose(Pose2d pose) {
    mPoseEstimator.resetPosition(swerveDrive.getRobotAngle(), swerveWheelModuleSubsystem.getSwerveModulePosition(), pose);
  }

}