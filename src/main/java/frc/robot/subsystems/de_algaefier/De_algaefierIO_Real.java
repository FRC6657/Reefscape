package frc.robot.subsystems.de_algaefier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.De_algaefier;

public class De_algaefierIO_Real implements De_algaefierIO {

  private SparkMax kPivot;
  private RelativeEncoder kEncoder;

  private PIDController pivotPID = new PIDController(1.3, 0, 0);

  public De_algaefierIO_Real() {

    kPivot = new SparkMax(Constants.CAN.AlgaeMotor.id, MotorType.kBrushless);
    kEncoder = kPivot.getEncoder();

    // configure the motor
    kPivot.configure(
        new SparkMaxConfig()
            .apply(new EncoderConfig().positionConversionFactor(1d / De_algaefier.pivotGearing))
            .smartCurrentLimit(De_algaefier.kPivotSupplyLimit)
            .idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(De_algaefierIOInputs inputs) {

    inputs.kSetpoint = Units.rotationsToDegrees(pivotPID.getSetpoint());
    inputs.kPosition = Units.rotationsToDegrees(kEncoder.getPosition());
    inputs.kVelocity = Units.rotationsToDegrees(kEncoder.getVelocity());

    inputs.kCurrent = kPivot.getOutputCurrent();
    inputs.kVoltage = kPivot.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.kTemp = kPivot.getMotorTemperature();

    kPivot.set(pivotPID.calculate(kEncoder.getPosition()));
  }

  @Override
  public void changeSetpoint(double rotations) {
    pivotPID.setSetpoint(MathUtil.clamp(rotations, De_algaefier.minAngle, De_algaefier.maxAngle));
  }
}
