package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIO_Sim implements ElevatorIO {

  public ElevatorSim elevatorSim;

  public double setpoint = Constants.Elevator.minHeight;
  public ProfiledPIDController pid =
      new ProfiledPIDController(
          150,
          0,
          0,
          new Constraints(30d / Constants.Elevator.stages, 15 / Constants.Elevator.stages));

  public ElevatorIO_Sim() {

    elevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(2),
            Constants.Elevator.gearing,
            Units.lbsToKilograms(8.716) * 3
                + Units.lbsToKilograms(4.661) * 2
                + Units.lbsToKilograms(5.74),
            Units.inchesToMeters(Constants.Elevator.sprocketPD / 2),
            Constants.Elevator.minHeight,
            Constants.Elevator.maxHeight / Constants.Elevator.stages,
            false,
            0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.kSetpoint = setpoint;
    inputs.kPosition = elevatorSim.getPositionMeters() * Constants.Elevator.stages;
    inputs.kVelocity = elevatorSim.getVelocityMetersPerSecond() * Constants.Elevator.stages;
    double pidEffort =
        pid.calculate(
            inputs.kPosition / Constants.Elevator.stages,
            inputs.kSetpoint / Constants.Elevator.stages);
    inputs.leaderMotorVoltage = MathUtil.clamp(pidEffort, -12, 12);
    elevatorSim.setInputVoltage(inputs.leaderMotorVoltage);
    elevatorSim.update(1 / Constants.mainLoopFrequency);

    Logger.recordOutput(
        "ElevatorSim/ProfileSetpoint", pid.getSetpoint().position * Constants.Elevator.stages);
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }
}
