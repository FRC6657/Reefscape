package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberIO_Real implements ClimberIO {

  // motor controller
  private SparkMax climberMotor;

  RelativeEncoder climberEncoder;

  // PID Controller
  private PIDController climberPID = new PIDController(0, 0, 0); // TODO Tune

  // store/log setpoints
  @AutoLogOutput(key = "Climber/Angle Setpoint")
  private double angleSetpoint = Constants.Climber.minRotations;

  @AutoLogOutput(key = "Climber/Speed Setpoint")
  private double speedSetpoint = 0;

  public ClimberIO_Real() {
    // motor configs
    climberMotor = new SparkMax(Constants.CAN.Climber.id, MotorType.kBrushless);
    climberMotor.setCANTimeout(250);
    SparkMaxConfig mConfig = new SparkMaxConfig();
    mConfig.inverted(false);
    mConfig.voltageCompensation(12);
    mConfig.smartCurrentLimit(Constants.Climber.currentLimit);
    mConfig.idleMode(IdleMode.kBrake);
    climberMotor.configure(mConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    changeSetpoint(Constants.Climber.minRotations);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    double pidOutput =
        climberPID.calculate(Units.rotationsToRadians(inputs.encoderRelPosition), angleSetpoint);
    climberMotor.setVoltage(pidOutput);

    inputs.encoderRelPosition = Units.rotationsToRadians(climberEncoder.getPosition());
    inputs.encoderVelocity = Units.rotationsToRadians(climberEncoder.getVelocity());

    inputs.position = Units.rotationsToRadians(climberMotor.getEncoder().getPosition());
    inputs.velocity = Units.rotationsToRadians(climberMotor.getEncoder().getVelocity());
    inputs.current = climberMotor.getOutputCurrent();
    inputs.voltage = climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.temp = climberMotor.getMotorTemperature();
    inputs.setpoint = angleSetpoint;
  }

  @Override
  public void changeSetpoint(double setpoint) {
    angleSetpoint = setpoint;
  }
}
