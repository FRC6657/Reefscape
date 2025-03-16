package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.ModuleConstants;
import java.util.Queue;

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
  private final VelocityVoltage drivePID = new VelocityVoltage(0.0);
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0);

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

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

    BaseStatusSignal.setUpdateFrequencyForAll(
        Swerve.odometryFrequency, drivePosition, turnPosition);

    // Set Status Signal Update Frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    // Optimize Bus Utilization
    drive.optimizeBusUtilization();
    turn.optimizeBusUtilization();

    // encoder
    //     .getAbsPositionFrame()
    //     .addCallback(
    //         frame -> {
    //           turn.setPosition(frame.getValue());
    //           CanandmagSettings stg =
    //               new CanandmagSettings()
    //                   .setPositionFramePeriod(0)
    //                   .setVelocityFramePeriod(0)
    //                   .setStatusFramePeriod(1);
    //           encoder.setSettings(stg, 0, 1);
    //         });

    CanandmagSettings stg =
        new CanandmagSettings()
            .setPositionFramePeriod(50)
            .setVelocityFramePeriod(0)
            .setStatusFramePeriod(1);
    encoder.setSettings(stg, 0, 1);

    turn.setPosition(encoder.getAbsPosition());

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drive.getPosition());
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turn.getPosition());
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

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositions =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
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

  @Override
  /** Reset the drive encoder to 0 */
  public void resetDriveEncoder() {
    drive.setPosition(0);
  }
}
