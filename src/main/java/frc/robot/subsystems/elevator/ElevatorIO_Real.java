package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import org.littletonrobotics.junction.Logger;

public class ElevatorIO_Real implements ElevatorIO {

  TalonFX leaderMotor = new TalonFX(CAN.Elevetor_Leader.id);
  TalonFX followMotor = new TalonFX(CAN.Elevator_Follower.id);

  VictorSPX algaeMotor = new VictorSPX(CAN.AlgaeMotor.id);

  private double kSetpoint = Constants.Elevator.minHeight;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  private double algaeArmMotorSetpoint = 0.0;
  private boolean algaeArmOut = false;
  private Timer algaeArmTimer = new Timer();

  public ElevatorIO_Real() {

    // Configure both motors
    var leaderConfigurator = leaderMotor.getConfigurator();
    var followConfigurator = followMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio =
        Constants.Elevator.gearing; // Sets default output to rotations
    motorConfigs.Slot0 = Constants.Elevator.motorSlot0; // PID Constants
    motorConfigs.CurrentLimits = Constants.Elevator.currentConfigs; // Current Limits
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfigs.MotionMagic = Constants.Elevator.kMotionMagicConfig;
    leaderConfigurator.apply(motorConfigs); // Configure leader motor
    followConfigurator.apply(motorConfigs); // Configure follow motor to the same thing
    followMotor.setControl(
        new Follower(
            CAN.Elevetor_Leader.id,
            false)); // Only difference with the follow motor configuration is this line

    // algae motor configure
    algaeMotor.configFactoryDefault();
    algaeMotor.setNeutralMode(NeutralMode.Brake);
    algaeMotor.setInverted(InvertType.None); // TODO: confirm

    // grab important numbers for logging
    var motorPostition = leaderMotor.getPosition();
    var motorVelocity = leaderMotor.getVelocity();
    var motorAcceleration = leaderMotor.getAcceleration();

    var leaderMotorTemp = leaderMotor.getDeviceTemp();
    var followMotorTemp = followMotor.getDeviceTemp();
    var leaderMotorVoltage = leaderMotor.getMotorVoltage();
    var followMotorVoltage = followMotor.getMotorVoltage();
    var leaderMotorCurrent = leaderMotor.getSupplyCurrent();
    var followMotorCurrent = followMotor.getSupplyCurrent();

    leaderMotorTemp.setUpdateFrequency(Constants.mainLoopFrequency / 4);
    followMotorTemp.setUpdateFrequency(Constants.mainLoopFrequency / 4);
    motorPostition.setUpdateFrequency(Constants.mainLoopFrequency);
    motorVelocity.setUpdateFrequency(Constants.mainLoopFrequency);
    motorAcceleration.setUpdateFrequency(Constants.mainLoopFrequency);
    leaderMotorVoltage.setUpdateFrequency(Constants.mainLoopFrequency);
    followMotorVoltage.setUpdateFrequency(Constants.mainLoopFrequency);
    leaderMotorCurrent.setUpdateFrequency(Constants.mainLoopFrequency);
    followMotorCurrent.setUpdateFrequency(Constants.mainLoopFrequency);

    var closedLoopReferenceSignal = leaderMotor.getClosedLoopReference();
    closedLoopReferenceSignal.setUpdateFrequency(Constants.mainLoopFrequency);

    // reduces CAN bus usage
    leaderMotor.optimizeBusUtilization();
    followMotor.optimizeBusUtilization();

    leaderMotor.setPosition(0);

    changeSetpoint(Constants.Elevator.minHeight);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.kSetpoint = kSetpoint;
    inputs.kPosition =
        Units.inchesToMeters(
            leaderMotor.getPosition().getValueAsDouble()
                * Constants.Elevator.sprocketPD
                * Constants.Elevator.stages
                * Math.PI);
    inputs.kVelocity =
        Units.inchesToMeters(
            leaderMotor.getVelocity().getValueAsDouble()
                * Constants.Elevator.sprocketPD
                * Constants.Elevator.stages
                * Math.PI);
    inputs.kAcceleration =
        Units.inchesToMeters(
            leaderMotor.getAcceleration().getValueAsDouble()
                * Constants.Elevator.sprocketPD
                * Constants.Elevator.stages
                * Math.PI);
    inputs.leaderMotorTemp = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.followMotorTemp = followMotor.getDeviceTemp().getValueAsDouble();
    inputs.leaderMotorCurrent = leaderMotor.getSupplyCurrent().getValueAsDouble();
    inputs.followMotorCurrent = followMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leaderMotorVoltage = leaderMotor.get() * RobotController.getBatteryVoltage();
    inputs.followMotorVoltage = followMotor.getMotorVoltage().getValueAsDouble();

    // leaderMotor.setControl(
    //     motionMagicVoltage.withPosition(
    //         Units.metersToInches(kSetpoint / Constants.Elevator.stages)
    //             / (Constants.Elevator.sprocketPD * Math.PI)));

    // followMotor.setControl(new Follower(CAN.Elevetor_Leader.id, false));

    runAlgaeArm(inputs.kPosition);

    inputs.algaeMotorVoltage = algaeMotor.getBusVoltage();
    inputs.algaeSetpoint = algaeArmMotorSetpoint;

    // Logging for motion magic internal variables for tuning purposes.
    Logger.recordOutput("Elevator/MotionMagicPosition", motionMagicVoltage.Position);
    Logger.recordOutput(
        "Elevator/MotionMagicSetpoint", leaderMotor.getClosedLoopReference().getValueAsDouble());
  }

  @Override
  public void changeSetpoint(double setpoint) {
    kSetpoint = setpoint;
  }

  // Automatically extends the algae arm when the elevator is up
  public void runAlgaeArm(double elevatorPosition) {

    // Stop the motor if it has been running for 0.5 seconds
    if (algaeArmTimer.hasElapsed(0.25)) {
      algaeArmMotorSetpoint = 0.0;
    }

    if (MathUtil.isNear(0, elevatorPosition, Units.inchesToMeters(0.5)) && algaeArmOut) {
      // If elevator is down and the arm is out bring the arm in
      algaeArmOut = false;
      algaeArmMotorSetpoint = -Constants.Elevator.kAlgaeStrength;
      algaeArmTimer.stop();
      algaeArmTimer.reset();
      algaeArmTimer.start();
    } else if (!algaeArmOut) {
      // If elevator is up and the arm is not out, bring the arm out
      algaeArmOut = true;
      algaeArmMotorSetpoint = Constants.Elevator.kAlgaeStrength;
      algaeArmTimer.stop();
      algaeArmTimer.reset();
      algaeArmTimer.start();
    }

    algaeMotor.set(ControlMode.PercentOutput, algaeArmMotorSetpoint);
  }
}
