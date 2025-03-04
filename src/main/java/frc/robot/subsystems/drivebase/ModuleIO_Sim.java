package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.ModuleConstants;

public class ModuleIO_Sim implements ModuleIO {

  private static final double loopPeriod = 0.02;
  private final ModuleConstants constants;

  private static final DCMotor driveMotor = DCMotor.getFalcon500(1);
  private static final DCMotor turnMotor = DCMotor.getFalcon500(1);

  private final TalonFX drive;

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double turnAppliedVolts = 0.0;

  private final VelocityVoltage drivePID = new VelocityVoltage(0.0);
  private final PIDController turnPID = new PIDController(100.0, 0.0, 0.0);

  public ModuleIO_Sim(ModuleConstants constants) {

    this.constants = constants;

    drive = new TalonFX(constants.driveID());
    drive.getConfigurator().apply(Constants.Swerve.driveConfig);

    turnPID.enableContinuousInput(-0.5, 0.5);

    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotor, 0.005, Constants.Swerve.driveRatio),
            driveMotor,
            0,
            0);

    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotor, 0.0004, Constants.Swerve.turnRatio),
            turnMotor,
            0,
            0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    var driveSimState = drive.getSimState();
    driveSimState.Orientation = ChassisReference.Clockwise_Positive;
    driveSim.setInput(driveSimState.getMotorVoltage());

    driveSim.update(loopPeriod);
    turnSim.update(loopPeriod);

    driveSimState.setRotorVelocity(
        (driveSim.getAngularVelocityRPM() / 60.0) * Constants.Swerve.driveRatio);

    inputs.prefix = constants.prefix();

    inputs.drivePositionMeters =
        driveSim.getAngularPositionRotations()
            * (Constants.Swerve.wheelRadiusMeters * 2.0 * Math.PI);
    inputs.driveVelocityMetersPerSec =
        (driveSim.getAngularVelocityRPM() / 60)
            * (Constants.Swerve.wheelRadiusMeters * 2.0 * Math.PI);
    inputs.driveAppliedVolts = driveSimState.getMotorVoltage();
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveSetpoint(double metersPerSecond) {
    drive.setControl(drivePID.withVelocity(metersPerSecond));
  }

  @Override
  public void setTurnSetpoint(Rotation2d rotation) {
    turnSim.setInputVoltage(
        MathUtil.clamp(
            turnPID.calculate(turnSim.getAngularPositionRotations(), rotation.getRotations()),
            -12.0,
            12.0));
  }
}
