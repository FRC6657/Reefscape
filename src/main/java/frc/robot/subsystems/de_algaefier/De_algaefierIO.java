package frc.robot.subsystems.de_algaefier;

import org.littletonrobotics.junction.AutoLog;

public interface De_algaefierIO {

  @AutoLog
  public static class De_algaefierIOInputs {
    public double kSetpoint = 0;
    public double kPosition = 0;
    public double kVelocity = 0;
    // public double kAcceleration = 0;

    public double kVoltage = 0;
    public double kCurrent = 0;
    public double kTemp = 0;
  }

  public default void updateInputs(De_algaefierIOInputs inputs) {}

  public default void changeSetpoint(double rotations){}
}
