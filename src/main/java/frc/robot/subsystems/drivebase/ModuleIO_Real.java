package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.ModuleConstants;

public class ModuleIO_Real implements ModuleIO {

  private final ModuleConstants constants;

  private final TalonFX drive;
  private final TalonFX turn;
  private final Canandmag encoder;

  private final BaseStatusSignal drivePosition;
  private final BaseStatusSignal driveVelocity;
  private final BaseStatusSignal driveAppliedVolts;
  private final BaseStatusSignal driveCurrent;
  private final BaseStatusSignal driveSupplyCurrent;

  private final BaseStatusSignal turnPosition;
  private final BaseStatusSignal turnVelocity;
  private final BaseStatusSignal turnAppliedVolts;
  private final BaseStatusSignal turnCurrent;

  private final MotionMagicVelocityVoltage drivePID = new MotionMagicVelocityVoltage(0.0);
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0);

  public ModuleIO_Real(ModuleConstants constants) {
    this.constants = constants;

    drive = new TalonFX(constants.driveID());
    turn = new TalonFX(constants.turnID());
    encoder = new Canandmag(constants.encoderID());

    drive.getConfigurator().apply(Constants.Swerve.driveConfig);
    turn.getConfigurator().apply(Constants.Swerve.turnConfig);

    drivePosition = drive.getPosition();
    driveVelocity = drive.getVelocity();
    driveAppliedVolts = drive.getMotorVoltage();
    driveCurrent = drive.getStatorCurrent();
    driveSupplyCurrent = drive.getSupplyCurrent();

    turnPosition = turn.getPosition();
    turnVelocity = turn.getVelocity();
    turnAppliedVolts = turn.getMotorVoltage();
    turnCurrent = turn.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100, drivePosition, turnPosition);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    turn.setPosition(encoder.getAbsPosition());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.prefix = constants.prefix();

    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(encoder.getAbsPosition());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
  }

  @Override
  public void setDriveSetpoint(double metersPerSecond) {
    if (metersPerSecond == 0 && MathUtil.isNear(0.0, driveVelocity.getValueAsDouble(), 0.1)) {
      drive.setControl(new VoltageOut(0));
    } else {
      drive.setControl(drivePID.withVelocity(metersPerSecond));
    }
  }

  @Override
  public void setTurnSetpoint(Rotation2d rotation) {
      turn.setControl(turnPID.withPosition(rotation.getRotations()));
  }

}
