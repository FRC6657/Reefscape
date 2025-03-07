package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.CAN;

public class GyroIO_Real implements GyroIO {

  private final Pigeon2 gyro; // Gryo
  private final StatusSignal<Angle> yaw; // Yaw
  private final StatusSignal<AngularVelocity> yawVelocity; // Pitch

  public GyroIO_Real() {

    gyro = new Pigeon2(CAN.Gyro.id);

    yaw = gyro.getYaw();
    yawVelocity = gyro.getAngularVelocityZWorld();

    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(50);
    yawVelocity.setUpdateFrequency(50);
    gyro.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    BaseStatusSignal.refreshAll(yaw, yawVelocity);

    inputs.yawPosition = new Rotation2d(yaw.getValue());
    inputs.yawVelocityRadPerSec = yawVelocity.getValue().in(RadiansPerSecond);
    inputs.timestamp = yaw.getTimestamp().getTime();
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    gyro.setYaw(yaw.getDegrees());
  }
}
