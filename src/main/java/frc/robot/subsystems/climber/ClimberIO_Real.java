package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimberIO_Real implements ClimberIO {

  // Motor
  private SparkMax motor;

  // PID Controller
  private PIDController pid = new PIDController(0, 0, 0); // TODO Tune

  // Log setpoint
  private double setpoint = Constants.ClimberConstants.minRotations;

  public ClimberIO_Real() {
    motor = new SparkMax(Constants.CAN.Climber.id, MotorType.kBrushless);
    motor.setCANTimeout(250);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.smartCurrentLimit(Constants.ClimberConstants.currentLimit);
    config.idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1d / Constants.ClimberConstants.gearing);
    config.encoder.velocityConversionFactor(1d / Constants.ClimberConstants.gearing);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor.getEncoder().setPosition(0);

    changeSetpoint(Constants.ClimberConstants.minRotations);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.position = -motor.getEncoder().getPosition();
    inputs.velocity = motor.getEncoder().getVelocity();
    inputs.current = motor.getOutputCurrent();
    inputs.voltage = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.temp = motor.getMotorTemperature();
    inputs.setpoint = setpoint;

    double pidOutput = pid.calculate(inputs.position, setpoint);
    Logger.recordOutput("Climber/PIDOutputVoltage", pidOutput);
    // motor.setVoltage(pidOutput);
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
