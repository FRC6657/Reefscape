package frc.robot.subsystems.de_algaefier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.De_algaefier;
import org.littletonrobotics.junction.Logger;

public class De_algaefierIO_Real implements De_algaefierIO {

  private SparkMax kPivot;
  private RelativeEncoder kEncoder;

  private double kSetpoint = 0;

  private ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          4, 0, 0, new Constraints(Units.degreesToRadians(200), Units.degreesToRadians(200)));


  public De_algaefierIO_Real() {

    kPivot = new SparkMax(Constants.CAN.AlgaeMotor.id, MotorType.kBrushless);
    kEncoder = kPivot.getEncoder();
    kEncoder.setPosition(0);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.smartCurrentLimit(De_algaefier.kPivotSupplyLimit);
    config.idleMode(IdleMode.kBrake);

    config.encoder.positionConversionFactor(1d / De_algaefier.pivotGearing);
    config.encoder.velocityConversionFactor(1d / De_algaefier.pivotGearing);

    // configure the motor
    kPivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    changeSetpoint(0);
  }

  @Override
  public void updateInputs(De_algaefierIOInputs inputs) {
    inputs.kSetpoint = kSetpoint;

    inputs.kPosition = Units.rotationsToDegrees(kEncoder.getPosition());
    inputs.kVelocity = Units.rotationsToDegrees(kEncoder.getVelocity());

    inputs.kCurrent = kPivot.getOutputCurrent();
    inputs.kVoltage = kPivot.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.kTemp = kPivot.getMotorTemperature();

    Logger.recordOutput("PIDTest", pivotPID.calculate(kEncoder.getPosition()));

    double pidOutput = pivotPID.calculate(inputs.kPosition, kSetpoint);
    kPivot.setVoltage(pidOutput);

    
  }

  @Override
  public void changeSetpoint(double rotations) {
    kSetpoint = rotations;
  }
}
