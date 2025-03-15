package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public Rotation2d yawPosition = new Rotation2d();
    public double yaw = 0.0;
    public double yawVelocityRadPerSec = 0.0;
    public double yawTimestamp = 0.0; // Timestamp of the latest yaw reading
    public double[] yawTimestamps = new double[] {};
    public Rotation2d[] yawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void setYaw(Rotation2d yaw) {}
}
