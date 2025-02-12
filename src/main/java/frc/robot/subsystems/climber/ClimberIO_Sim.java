package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberIO_Sim implements ClimberIO {
  // voltage tracking
  private double voltage = 0;

  // setpoint tracking
  @AutoLogOutput(key = "Climber/Speed Setpoint")
  private double speedSetpoint = 0;

  @AutoLogOutput(key = "Climber/Angle Setpoint")
  private double angleSetpoint = Constants.Climber.minRotations;

  private DCMotorSim climberSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.0001, Constants.Climber.gearing),
          DCMotor.getNEO(1));

  private PIDController climberPID = new PIDController(96d / 360, 0, 0);

  public ClimberIO_Sim() {
    climberSim.setState(Units.degreesToRadians(Constants.Climber.maxRotations), 0);
    climberPID.setTolerance(2);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    updatePID(); // update PID Controller

    // update sim
    climberSim.update(1 / Constants.mainLoopFrequency);

    // inputs
    inputs.position = climberSim.getAngularPositionRotations() * 360; // degrees
    inputs.velocity = climberSim.getAngularVelocityRPM() * (360d / 60); // degrees
    inputs.temp = 0; // celcius
    inputs.voltage = voltage; // volts
    inputs.current = climberSim.getCurrentDrawAmps(); // amps
    inputs.atSetpoint = climberPID.atSetpoint();
    inputs.setpoint = angleSetpoint;
  }

  /**
   * Change the setpoint of the shooter pivot
   *
   * @param angleDegrees The new setpoint in degrees
   *     <p>Acceptable Range: Find it.
   */
  @Override
  public void changeSetpoint(double angleDegrees) {
    angleSetpoint = angleDegrees;
  }

  private void updatePID() {
    double climberPIDEffort =
        climberPID.calculate(climberSim.getAngularPositionRotations() * 360, angleSetpoint);
    voltage = MathUtil.clamp(climberPIDEffort, -12, 12);
    climberSim.setInput(voltage);
  }
}
