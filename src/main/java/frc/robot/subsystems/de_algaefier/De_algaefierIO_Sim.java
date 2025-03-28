package frc.robot.subsystems.de_algaefier;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.De_algaefier;

public class De_algaefierIO_Sim implements De_algaefierIO {

  private PIDController pivotPID = new PIDController(10, 0, 0);

  private DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNeo550(1), 0.0001, Constants.De_algaefier.pivotGearing),
          DCMotor.getNeo550(1));

  public De_algaefierIO_Sim() {}

  @Override
  public void updateInputs(De_algaefierIOInputs inputs) {

    double output = pivotPID.calculate(motor.getAngularPosition().in(Rotation));
    motor.setInputVoltage(MathUtil.clamp(output, -12, 12));
    motor.update(0.02);

    inputs.kSetpoint = pivotPID.getSetpoint();
    inputs.kPosition = motor.getAngularPosition().in(Rotations);
    inputs.kVelocity = motor.getAngularVelocity().in(RotationsPerSecond);
    inputs.kVoltage = output * RobotController.getBatteryVoltage();
  }

  @Override
  public void changeSetpoint(double rotations) {
    pivotPID.setSetpoint(MathUtil.clamp(rotations, De_algaefier.minAngle, De_algaefier.maxAngle));
  }
}
