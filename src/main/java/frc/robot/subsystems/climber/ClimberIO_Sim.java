package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ClimberIO_Sim implements ClimberIO {
  // voltage tracking
  private double voltage = 0;

  private double angleSetpoint = Constants.ClimberConstants.minRotations;

  private DCMotorSim climberSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNEO(1), 0.0001, Constants.ClimberConstants.gearing),
          DCMotor.getNEO(1));

  public ClimberIO_Sim() {
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    // update sim
    climberSim.update(1 / Constants.mainLoopFrequency);

    // inputs
    inputs.position = climberSim.getAngularPositionRotations(); // rot
    inputs.velocity = climberSim.getAngularVelocityRPM() * (1d / 60); // rot/s
    inputs.temp = 0; // celcius
    inputs.voltage = voltage; // volts
    inputs.current = climberSim.getCurrentDrawAmps(); // amps
    inputs.setpoint = angleSetpoint;
  }

  @Override
  public void setVoltage(double volts) {
    voltage = MathUtil.clamp(volts, -12, 12);
    climberSim.setInput(voltage);
  }
}
