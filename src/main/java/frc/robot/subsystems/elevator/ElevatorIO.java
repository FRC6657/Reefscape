package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double kSetpoint = 0.0; // Meters
    public double kPosition = 0.0; // Meters
    public double kVelocity = 0.0; // Meters per second
    public double kAcceleration = 0.0; // Meters per second per second.

    public double leaderMotorTemp = 0.0; // Celcius
    public double leaderMotorVoltage = 0.0; // Volts
    public double leaderMotorCurrent = 0.0; // Amps

    public double followMotorTemp = 0.0; // Celcius
    public double followMotorVoltage = 0.0; // Volts
    public double followMotorCurrent = 0.0; // Amps
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void changeSetpoint(double setpoint) {}
}
