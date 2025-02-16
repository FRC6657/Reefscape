package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants.CameraInfo;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class ApriltagCameraIO_Sim implements ApriltagCameraIO {

  private final VisionSystemSim sim;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;

  public ApriltagCameraIO_Sim(CameraInfo cameraInfo) {

    this.sim = new VisionSystemSim(cameraInfo.cameraName);
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(cameraInfo.cameraRes[0], cameraInfo.cameraRes[1], cameraInfo.diagFOV);
    cameraProp.setCalibError(0, 0);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setExposureTimeMs(20);
    cameraProp.setFPS(50);
    cameraProp.setLatencyStdDevMs(5.0);

    this.camera = new PhotonCamera(cameraInfo.cameraName);
    this.cameraSim = new PhotonCameraSim(camera, cameraProp);
    cameraSim.enableDrawWireframe(true);
    cameraSim.setMaxSightRange(7);
    sim.addCamera(cameraSim, cameraInfo.robotToCamera);
    try {
      var field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addAprilTags(field);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(AprilTagCameraIOInputs inputs) {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      inputs.result = results.get(results.size() - 1);
    } else {
      inputs.result = new PhotonPipelineResult();
    }
  }

  @Override
  public void updateSimPose(Pose2d robotPose) {
    sim.update(robotPose);
  }
}
