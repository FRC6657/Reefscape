package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double position = 0.0; // Degrees
    public double velocity = 0.0; // Degrees per second
    public double acceleration = 0.0;
    public double temp = 0.0; // Celcius
    public double voltage = 0.0; // Volts
    public double current = 0.0; // Amps
    public double setpoint = 0.0;
    public boolean atSetpoint = false;

    public double encoderRelPosition = 0.0; // Rad
    public double encoderVelocity = 0.0; // Rad/s
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void changeSetpoint(double angleDegrees) {}
}
