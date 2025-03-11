package frc.robot.subsystems.outtake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class OuttakeIO_Sim implements OuttakeIO {

  // Voltage variable
  private double outtakeVoltage = 0;

  // Setpoint variable
  private double rpmSetpoint = 0;

  // Simulated motor
  private DCMotorSim outtakeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, Constants.Outtake.gearing),
          DCMotor.getFalcon500(1));

  public OuttakeIO_Sim() {}

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    // Inputs
    inputs.kVelocity = outtakeSim.getAngularVelocityRPM(); // RPM
    inputs.kTemp = 0; // Celcius (32º F)
    inputs.kVoltage = outtakeVoltage; // Voltage (set to 0 at start)
    inputs.kCurrent = outtakeSim.getCurrentDrawAmps(); // Amps
    inputs.kSetpoint = rpmSetpoint;
  }
  /**
   * Change Setpoint for RPM
   *
   * @param rpm New Setpoint in RPM
   *     <p>Acceptable Range: [-3190, 3190] Positive RPM shoots?
   */
  @Override
  public void changeSetpoint(double rpm) {
    rpmSetpoint = rpm;
  }
}
