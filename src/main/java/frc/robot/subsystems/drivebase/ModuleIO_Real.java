package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.Motors;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.ModuleInformation;

public class ModuleIO_Real implements ModuleIO {

  // Module Hardware
  private TalonFX drive;
  private TalonFX turn;
  private Canandmag encoder;

  // Module Control Variables
  private VelocityVoltage driveControl;
  private PositionVoltage turnControl;

  private StatusSignal<Voltage> driveApplied; // Volts
  private StatusSignal<Current> driveStatorCurrent; // Amps
  private StatusSignal<Current> driveSupplyCurrent; // Amps
  private StatusSignal<Angle> drivePosition; // Mechanism Rotations
  private StatusSignal<AngularVelocity> driveVelocity; // Mechanism Rotations per Second
  private StatusSignal<AngularAcceleration> driveAcceleration; // Mecanism Roations per Second^2
  private StatusSignal<Temperature> driveTemp; // Celcius

  private StatusSignal<Voltage> turnApplied; // Volts
  private StatusSignal<Current> turnStatorCurrent; // Amps
  private StatusSignal<Current> turnSupplyCurrent; // Amps
  private StatusSignal<Angle> turnPosition; // Mechanism Rotations
  private StatusSignal<AngularVelocity> turnVelocity; // Mechanism Rotations per Second
  private StatusSignal<AngularAcceleration> turnAcceleration; // Mecanism Roations per Second^2
  private StatusSignal<Temperature> turnTemp; // Celcius

  public ModuleIO_Real(ModuleInformation moduleInformation) {

    // Assign Module Hardware CAN IDs
    drive = new TalonFX(moduleInformation.driveID);
    turn = new TalonFX(moduleInformation.turnID);
    encoder = new Canandmag(moduleInformation.encoderID);

    // Set Default Control Values
    driveControl = new VelocityVoltage(0);
    turnControl = new PositionVoltage(turn.getPosition().getValueAsDouble());

    // Drive Motor Configuration
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 80; // reduced from 80
    driveConfig.CurrentLimits.SupplyCurrentLimit = 65; // reduced from 65 to 40 for testing
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = Swerve.DriveGearing.L3.reduction;
    driveConfig.Slot0.kS = 1;
    driveConfig.Slot0.kA = 0;
    driveConfig.Slot0.kV = (12d / (Motors.FalconRPS * Swerve.DriveGearing.L3.reduction));
    driveConfig.Slot0.kP = 2.25;
    driveConfig.Slot0.kD = 0;
    if (moduleInformation.name == "Back Left ") {
      driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    } else {
      driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    drive.getConfigurator().apply(driveConfig);

    // Drive Motor Status Signals
    driveApplied = drive.getMotorVoltage();
    driveStatorCurrent = drive.getStatorCurrent();
    driveSupplyCurrent = drive.getSupplyCurrent();
    drivePosition = drive.getPosition();
    driveVelocity = drive.getVelocity();
    driveAcceleration = drive.getAcceleration();
    driveTemp = drive.getDeviceTemp();

    driveApplied.setUpdateFrequency(Constants.mainLoopFrequency);
    driveStatorCurrent.setUpdateFrequency(Constants.mainLoopFrequency / 2);
    driveSupplyCurrent.setUpdateFrequency(Constants.mainLoopFrequency / 2);
    drivePosition.setUpdateFrequency(Constants.mainLoopFrequency);
    driveVelocity.setUpdateFrequency(Constants.mainLoopFrequency);
    driveAcceleration.setUpdateFrequency(Constants.mainLoopFrequency);
    driveTemp.setUpdateFrequency(Constants.mainLoopFrequency / 4);

    drive.optimizeBusUtilization();

    // Turn Motor Configuration
    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnConfig.CurrentLimits.StatorCurrentLimit = 40;
    turnConfig.CurrentLimits.SupplyCurrentLimit = 30;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Feedback.SensorToMechanismRatio = Swerve.TurnGearing;
    turnConfig.Slot0.kS = 0;
    turnConfig.Slot0.kA = 0;
    turnConfig.Slot0.kV = (12d / (Motors.FalconRPS * Swerve.TurnGearing));
    turnConfig.Slot0.kP = 150;
    turnConfig.Slot0.kD = 0;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    turn.getConfigurator().apply(turnConfig);

    // Turn Motor Status Signals
    turnApplied = turn.getMotorVoltage();
    turnStatorCurrent = turn.getStatorCurrent();
    turnSupplyCurrent = turn.getSupplyCurrent();
    turnPosition = turn.getPosition();
    turnVelocity = turn.getVelocity();
    turnAcceleration = turn.getAcceleration();
    turnTemp = turn.getDeviceTemp();

    turnApplied.setUpdateFrequency(Constants.mainLoopFrequency);
    turnStatorCurrent.setUpdateFrequency(Constants.mainLoopFrequency / 2);
    turnSupplyCurrent.setUpdateFrequency(Constants.mainLoopFrequency / 2);
    turnPosition.setUpdateFrequency(Constants.mainLoopFrequency);
    turnVelocity.setUpdateFrequency(Constants.mainLoopFrequency);
    turnAcceleration.setUpdateFrequency(Constants.mainLoopFrequency);
    turnTemp.setUpdateFrequency(Constants.mainLoopFrequency / 4);

    turn.optimizeBusUtilization();

    turn.setPosition(encoder.getAbsPosition()); // Sync the Turn Motor Encoder with the ABS Encoder.
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    inputs.driveApplied = drive.get() * RobotController.getBatteryVoltage();
    inputs.driveStatorCurrent = driveStatorCurrent.getValueAsDouble();
    inputs.driveSupplyCurrent = driveSupplyCurrent.getValueAsDouble();
    inputs.drivePosition = drivePosition.getValueAsDouble() * Swerve.WheelDiameter * Math.PI;
    inputs.driveVelocity = driveVelocity.getValueAsDouble() * Swerve.WheelDiameter * Math.PI;
    inputs.driveAcceleration =
        driveAcceleration.getValueAsDouble() * Swerve.WheelDiameter * Math.PI;
    inputs.driveTemp = driveTemp.getValueAsDouble();

    inputs.turnApplied = turnApplied.getValueAsDouble();
    inputs.turnStatorCurrent = turnStatorCurrent.getValueAsDouble();
    inputs.turnSupplyCurrent = turnSupplyCurrent.getValueAsDouble();
    inputs.turnPosition = turnPosition.getValueAsDouble() * 2 * Math.PI;
    inputs.turnVelocity = turnVelocity.getValueAsDouble() * 2 * Math.PI;
    inputs.turnAcceleration = turnAcceleration.getValueAsDouble() * 2 * Math.PI;
    inputs.turnTemp = turnTemp.getValueAsDouble();

    inputs.encoderAbsPosition = encoder.getAbsPosition() * 2 * Math.PI;
    inputs.encoderRelPosition = encoder.getPosition() * 2 * Math.PI;
    inputs.encoderVelocity = encoder.getVelocity() * 2 * Math.PI;
  }

  @Override
  public void changeDriveSetpoint(double mps) {
    drive.setControl(driveControl.withSlot(0).withVelocity(mps / (Swerve.WheelDiameter * Math.PI)));
  }

  @Override
  public void changeTurnSetpoint(double rad) {
    turn.setControl(turnControl.withSlot(0).withPosition(rad / (2 * Math.PI)));
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        drive.getPosition().getValueAsDouble() * Swerve.WheelDiameter * Math.PI,
        new Rotation2d(turn.getPosition().getValueAsDouble() * 2 * Math.PI));
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        drive.getVelocity().getValueAsDouble() * Swerve.WheelDiameter * Math.PI,
        new Rotation2d(turn.getPosition().getValueAsDouble() * 2 * Math.PI));
  }
}
