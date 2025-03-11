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

    gyro = new Pigeon2(CAN.Gyro.id); // Assign CAN ID to Gyro

    // Seed status signals
    yaw = gyro.getYaw();
    yawVelocity = gyro.getAngularVelocityZWorld();

    gyro.getConfigurator().apply(new Pigeon2Configuration()); // Reset Factory Defaults
    gyro.setYaw(0); // Zero Gyro

    BaseStatusSignal.setUpdateFrequencyForAll(50, yaw, yawVelocity); // Set update frequency
    gyro.optimizeBusUtilization(); // Turn down all other status frames we dont use
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    BaseStatusSignal.refreshAll(yaw, yawVelocity); // Pull Latest Data

    // Assign Inputs
    inputs.yawPosition = new Rotation2d(yaw.getValue()); // Normalized Yaw
    inputs.yaw = yaw.getValueAsDouble(); // Raw Yaw
    inputs.yawVelocityRadPerSec = yawVelocity.getValue().in(RadiansPerSecond); // Yaw Velocity
    inputs.timestamp = yaw.getTimestamp().getTime(); // Gyro Reading Timestamp
  }

  /** Set the yaw of the gyro */
  @Override
  public void setYaw(Rotation2d yaw) {
    gyro.setYaw(yaw.getDegrees());
  }
}
