// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraInfo;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class ApriltagCamera {

  private final ApriltagCameraIO io;
  private final AprilTagCameraIOInputsAutoLogged inputs = new AprilTagCameraIOInputsAutoLogged();

  private PhotonPoseEstimator poseEstimator;
  private final CameraInfo cameraInfo;

  private Pose3d latestPose = new Pose3d();
  private Matrix<N3, N1> stdDevs =
      VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private double latestTimestamp = -1;

  public ApriltagCamera(ApriltagCameraIO io, CameraInfo cameraInfo) {
    this.io = io;
    this.cameraInfo = cameraInfo;

    poseEstimator =
        new PhotonPoseEstimator(
            Constants.VisionConstants.kReefTagLayout,
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            cameraInfo.robotToCamera);
  }

  public void updateInputs(double headingTimestamp, Rotation2d robotHeading) {
    io.updateInputs(inputs);

    poseEstimator.addHeadingData(headingTimestamp, robotHeading);
    var result = poseEstimator.update(inputs.result);

    if (result.isPresent()) {
      latestPose = result.get().estimatedPose;
      latestTimestamp = result.get().timestampSeconds;
      stdDevs = getEstimationStdDevs(latestPose.toPose2d(), inputs.result);

      Translation2d[] tagCorners = new Translation2d[inputs.result.targets.size() * 4];
      Pose3d[] tagPoses = new Pose3d[inputs.result.targets.size()];

      for (int i = 0; i < inputs.result.targets.size(); i++) {
        tagPoses[i] =
            poseEstimator
                .getFieldTags()
                .getTagPose(inputs.result.targets.get(i).fiducialId)
                .orElse(new Pose3d());
      }

      int tagIndex = 0;
      for (var tag : inputs.result.targets) {
        for (var corner : tag.getDetectedCorners()) {
          tagCorners[tagIndex] = new Translation2d(corner.x, corner.y);
          tagIndex++;
        }
      }

      Logger.recordOutput(
          "Vision/ApriltagCameras/" + cameraInfo.cameraName + "/Corners", tagCorners);
      Logger.recordOutput(
          "Vision/ApriltagCameras/" + cameraInfo.cameraName + "/TagPoses", tagPoses);

    } else {
      latestPose = new Pose3d(new Translation3d(100, 100, 100), new Rotation3d());
      stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      Logger.recordOutput(
          "Vision/ApriltagCameras/" + cameraInfo.cameraName + "/Corners", new Translation2d[] {});
    }

    Logger.processInputs("Vision/ApriltagCameras/" + cameraInfo.cameraName + "/Inputs", inputs);
    Logger.recordOutput("Vision/ApriltagCameras/" + cameraInfo.cameraName + "/Pose", latestPose);
  }

  public Pose3d getEstimatedPose() {
    return latestPose;
  }

  private Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult result) {
    var estStdDevs = VisionConstants.singleTagStdDev;
    var targets = result.targets;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.multiTagStdDev;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4) estStdDevs = VisionConstants.singleTagStdDev;
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public Matrix<N3, N1> getLatestStdDevs() {
    return stdDevs;
  }

  public double getLatestTimestamp() {
    return latestTimestamp;
  }

  public void updateSimPose(Pose2d robotPose) {
    io.updateSimPose(robotPose);
  }
}
