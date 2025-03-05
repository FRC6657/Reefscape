package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public Rotation2d yawPosition = new Rotation2d();
    public Rotation3d heading = new Rotation3d();
    public double yawVelocityRadPerSec = 0.0;
    public double timestamp = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
