package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {
    public double kSetpoint = 0.0;
    public double kVelocity = 0.0;
    public double kTemp = 0.0;
    public double kVoltage = 0.0;
    public double kCurrent = 0.0;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default boolean getBeamBroken() {
    return true;
  }

  public default void changeSetpoint(double setpoint) {}
}
