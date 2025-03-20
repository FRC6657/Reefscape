package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraInfo;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class ApriltagCameraIO_Sim extends ApriltagCameraIO_Real {

  private static VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  public ApriltagCameraIO_Sim(CameraInfo cameraInfo, Supplier<Pose2d> poseSupplier) {
    super(cameraInfo);
    this.poseSupplier = poseSupplier;
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.kReefTagLayout);
    }
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(cameraInfo.cameraRes[0], cameraInfo.cameraRes[1], cameraInfo.diagFOV);
    cameraProp.setCalibError(0, 0);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setExposureTimeMs(20);
    cameraProp.setFPS(50);
    cameraProp.setLatencyStdDevMs(5.0);
    cameraSim = new PhotonCameraSim(camera, cameraProp, VisionConstants.kReefTagLayout);
    visionSim.addCamera(cameraSim, cameraInfo.robotToCamera);
  }

  @Override
  public void updateInputs(ApriltagCameraIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
