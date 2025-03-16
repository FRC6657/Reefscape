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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.De_algaefier;
import org.littletonrobotics.junction.Logger;

public class De_algaefierIO_Real implements De_algaefierIO {

  private SparkMax kPivot;
  private RelativeEncoder kEncoder;

  private PIDController pivotPID = new PIDController(1.3, 0, 0);

  public De_algaefierIO_Real() {

    kPivot = new SparkMax(Constants.CAN.AlgaeMotor.id, MotorType.kBrushless);
    kEncoder = kPivot.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.smartCurrentLimit(De_algaefier.kPivotSupplyLimit);
    config.idleMode(IdleMode.kBrake);

    config.encoder.positionConversionFactor(1d / De_algaefier.pivotGearing);
    config.encoder.velocityConversionFactor(1d / De_algaefier.pivotGearing);

    // config.signals.absoluteEncoderPositionAlwaysOn(false);
    // config.signals.absoluteEncoderPositionAlwaysOn(false);
    // config.signals.analogPositionAlwaysOn(false);
    // config.signals.analogVelocityAlwaysOn(false);
    // config.signals.appliedOutputPeriodMs(50);
    // config.signals.busVoltagePeriodMs(50);
    // config.signals.externalOrAltEncoderPositionAlwaysOn(false);
    // config.signals.externalOrAltEncoderVelocityAlwaysOn(false);
    // config.signals.iAccumulationAlwaysOn(false);
    // config.signals.motorTemperaturePeriodMs(100);
    // config.signals.outputCurrentPeriodMs(50);
    // config.signals.primaryEncoderPositionPeriodMs(50);
    // config.signals.primaryEncoderVelocityPeriodMs(50);

    // configure the motor
    kPivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    changeSetpoint(0);
  }

  @Override
  public void updateInputs(De_algaefierIOInputs inputs) {

    inputs.kPosition = Units.rotationsToDegrees(kEncoder.getPosition());
    inputs.kVelocity = Units.rotationsToDegrees(kEncoder.getVelocity());

    inputs.kCurrent = kPivot.getOutputCurrent();
    inputs.kVoltage = kPivot.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.kTemp = kPivot.getMotorTemperature();

    Logger.recordOutput("PIDTest", pivotPID.calculate(kEncoder.getPosition()));

    // kPivot.set(pivotPID.calculate(kEncoder.getPosition()));
    inputs.kSetpoint =
        Units.rotationsToDegrees(
            pivotPID
                .getSetpoint()); // I think it would be optimal to move this line to directly after
    // PID.calculate
  }

  @Override
  public void changeSetpoint(double rotations) {
    pivotPID.setSetpoint(MathUtil.clamp(rotations, De_algaefier.minAngle, De_algaefier.maxAngle));
  }
}
