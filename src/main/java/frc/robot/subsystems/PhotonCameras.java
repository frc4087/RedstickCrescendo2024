// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.VisionConstants;
//import frc.robot.subsystems.PhotonCameras;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// public class PhotonCameras extends ProfiledPIDSubsystem {
//   /** Creates a new PhotonVision. */
//   public PhotonCameras() {
//     super(
//         // The ProfiledPIDController used by the subsystem
//         new ProfiledPIDController(
//             0,
//             0,
//             0,
//             // The motion profile constraints
//             new TrapezoidProfile.Constraints(0, 0)));
//   }

//   @Override
//   protected void useOutput(double output, State setpoint) {
//     // TODO Auto-generated method stub
//     throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
//   }

//   @Override
//   protected double getMeasurement() {
//     // TODO Auto-generated method stub
//     throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
//   }
// }

public class PhotonCameras {
  private final PhotonCamera mPhotonCameraFront;
  //private final PhotonCamera mPhotonCameraBack;
  private PhotonPoseEstimator mPhotonPoseEstimatorFront;
  //private PhotonPoseEstimator mPhotonPoseEstimatorBack;

   public PhotonCameras() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        mPhotonCameraFront = new PhotonCamera(VisionConstants.CAMERA_NAME_FRONT);
        //mPhotonCameraBack = new PhotonCamera(VisionConstants.CAMERA_NAME_BACK);

        //try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the
            // field.
            AprilTagFieldLayout fieldLayout =
                    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
            // Create pose estimator
            mPhotonPoseEstimatorFront =
                    new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO,
                            mPhotonCameraFront, VisionConstants.ROBOT_TO_CAM_VEC_FRONT);
            mPhotonPoseEstimatorFront
                    .setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

            // mPhotonPoseEstimatorBack =
            //         new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP,
            //                 mPhotonCameraBack, VisionConstants.ROBOT_TO_CAM_VEC_BACK);
            // mPhotonPoseEstimatorBack
            //         .setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
         //}
        //catch (IOException e) {
        //     // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we
        //     // don't know
        //     // where the tags are.
        //     DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        //     mPhotonPoseEstimatorFront = null;
        //    // mPhotonPoseEstimatorBack = null;
        // }
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose from the FRONT CAMERA with an estimated pose, the timestamp,
     *         and targets used to create the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront(Pose2d prevEstimatedRobotPose) {
        if (mPhotonPoseEstimatorFront == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        mPhotonPoseEstimatorFront.setReferencePose(prevEstimatedRobotPose);
        return mPhotonPoseEstimatorFront.update();
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose from the BACK CAMERA with an estimated pose, the timestamp, and
     *         targets used to create the estimate
     */
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBack(Pose2d prevEstimatedRobotPose) {
    //     if (mPhotonPoseEstimatorBack == null) {
    //         // The field layout failed to load, so we cannot estimate poses.
    //         return Optional.empty();
    //     }
    //     mPhotonPoseEstimatorBack.setReferencePose(prevEstimatedRobotPose);
    //     return mPhotonPoseEstimatorBack.update();
    // }

}