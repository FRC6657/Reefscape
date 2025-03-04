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

  // Module Specific Constants
  private final ModuleConstants constants;

  // Module Hardware
  private final TalonFX drive;
  private final TalonFX turn;
  private final Canandmag encoder;

  // Useful Status Signals
  private final BaseStatusSignal drivePosition;
  private final BaseStatusSignal driveVelocity;
  private final BaseStatusSignal driveAppliedVolts;
  private final BaseStatusSignal driveCurrent;
  private final BaseStatusSignal driveSupplyCurrent;

  private final BaseStatusSignal turnPosition;
  private final BaseStatusSignal turnVelocity;
  private final BaseStatusSignal turnAppliedVolts;
  private final BaseStatusSignal turnCurrent;

  // Control Signals
  private final MotionMagicVelocityVoltage drivePID = new MotionMagicVelocityVoltage(0.0);
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0);

  public ModuleIO_Real(ModuleConstants constants) {
    this.constants = constants;

    // Assign Hardware
    drive = new TalonFX(constants.driveID());
    turn = new TalonFX(constants.turnID());
    encoder = new Canandmag(constants.encoderID());

    // Configure Motors
    drive.getConfigurator().apply(Constants.Swerve.driveConfig);
    turn.getConfigurator().apply(Constants.Swerve.turnConfig);

    // Assign Status Signals
    drivePosition = drive.getPosition();
    driveVelocity = drive.getVelocity();
    driveAppliedVolts = drive.getMotorVoltage();
    driveCurrent = drive.getStatorCurrent();
    driveSupplyCurrent = drive.getSupplyCurrent();

    turnPosition = turn.getPosition();
    turnVelocity = turn.getVelocity();
    turnAppliedVolts = turn.getMotorVoltage();
    turnCurrent = turn.getStatorCurrent();

    // Set Status Signal Update Frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    // Optimize Bus Utilization
    drive.optimizeBusUtilization();
    turn.optimizeBusUtilization();

    // Zero Turn Motor Encoder
    turn.setPosition(encoder.getAbsPosition());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // Refresh Status Signals
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

    // Update Inputs
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
    // If the robot is stopped, set the drive to 0 volts
    if (metersPerSecond == 0 && MathUtil.isNear(0.0, driveVelocity.getValueAsDouble(), 0.1)) {
      drive.setControl(new VoltageOut(0));
    } else { // Otherwise, set the drive to the desired velocity
      drive.setControl(drivePID.withVelocity(metersPerSecond));
    }
  }

  @Override
  public void setTurnSetpoint(Rotation2d rotation) {
    // Set the module rotation to the desired position
    turn.setControl(turnPID.withPosition(rotation.getRotations()));
  }
}
