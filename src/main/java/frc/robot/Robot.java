// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO_Real;
import frc.robot.subsystems.climber.ClimberIO_Sim;
import frc.robot.subsystems.de_algaefier.De_algaefier;
import frc.robot.subsystems.de_algaefier.De_algaefierIO_Real;
import frc.robot.subsystems.de_algaefier.De_algaefierIO_Sim;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO_Sim;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO_Real;
import frc.robot.subsystems.elevator.ElevatorIO_Sim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO_Real;
import frc.robot.subsystems.intake.IntakeIO_Sim;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO_Real;
import frc.robot.subsystems.outtake.OuttakeIO_Sim;
import frc.robot.subsystems.vision.ApriltagCameraIO_Real;
import frc.robot.subsystems.vision.ApriltagCameraIO_Sim;
import frc.robot.subsystems.vision.ApriltagCameras;
import frc.robot.util.DriveToPose;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandGenericHID operator = new CommandGenericHID(1);
  private CommandXboxController debug = new CommandXboxController(2);

  private final ApriltagCameras cameras;
  private final Swerve swerve;
  private final Elevator elevator;
  private final Intake intake;
  private final Outtake outtake;
  private final De_algaefier dealg;
  private final Climber climber;

  private final Superstructure superstructure;
  private final AutoFactory autoFactory;

  private LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  @AutoLogOutput(key = "RobotStates/SlowMode")
  private boolean slowMode = false;

  public Robot() {

    DriverStation.silenceJoystickConnectionWarning(true);

    swerve =
        new Swerve(
            RobotBase.isReal() ? new GyroIO_Real() : new GyroIO() {},
            new ModuleIO[] {
              RobotBase.isReal()
                  ? new ModuleIO_Real(Constants.Swerve.frontLeftModule)
                  : new ModuleIO_Sim(Constants.Swerve.frontLeftModule),
              RobotBase.isReal()
                  ? new ModuleIO_Real(Constants.Swerve.frontRightModule)
                  : new ModuleIO_Sim(Constants.Swerve.frontRightModule),
              RobotBase.isReal()
                  ? new ModuleIO_Real(Constants.Swerve.backLeftModule)
                  : new ModuleIO_Sim(Constants.Swerve.backLeftModule),
              RobotBase.isReal()
                  ? new ModuleIO_Real(Constants.Swerve.backRightModule)
                  : new ModuleIO_Sim(Constants.Swerve.backRightModule)
            });

    cameras =
        new ApriltagCameras(
            swerve::addVisionMeasurement,
            RobotBase.isReal() || replay
                ? new ApriltagCameraIO_Real(VisionConstants.WhiteReefInfo)
                : new ApriltagCameraIO_Sim(VisionConstants.WhiteReefInfo, swerve::getPose),
            RobotBase.isReal() || replay
                ? new ApriltagCameraIO_Real(VisionConstants.BlackReefInfo)
                : new ApriltagCameraIO_Sim(VisionConstants.BlackReefInfo, swerve::getPose));

    elevator = new Elevator(RobotBase.isReal() ? new ElevatorIO_Real() : new ElevatorIO_Sim());
    intake = new Intake(RobotBase.isReal() ? new IntakeIO_Real() : new IntakeIO_Sim());
    outtake = new Outtake(RobotBase.isReal() ? new OuttakeIO_Real() : new OuttakeIO_Sim());
    dealg =
        new De_algaefier(RobotBase.isReal() ? new De_algaefierIO_Real() : new De_algaefierIO_Sim());
    climber = new Climber(RobotBase.isReal() ? new ClimberIO_Real() : new ClimberIO_Sim());
    superstructure = new Superstructure(swerve, elevator, outtake, intake, dealg, climber);

    autoFactory =
        new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);

    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("CenterL4", superstructure.CenterL4(autoFactory).cmd());
    // autoChooser.addOption("3 Algae", superstructure.ThreeAlgae(autoFactory).cmd());
    // autoChooser.addOption("Test", superstructure.DirectionTest(autoFactory, false).cmd());
    autoChooser.addOption("3 Piece", superstructure.L4_3Piece(autoFactory, false).cmd());
    autoChooser.addOption(
        "Processor 3 Piece", superstructure.L4_3Piece(autoFactory, true).cmd().withTimeout(15));
    // autoChooser.addOption("Trough 3 Piece", superstructure.Trough_3Piece(autoFactory,
    // false).cmd());
    // autoChooser.addOption(
    //     "Trough 3 Piece Procesor", superstructure.Trough_3Piece(autoFactory, true).cmd());
  }

  public static boolean replay = false;

  @Override
  public void robotInit() {
    Logger.recordMetadata("Arborbotics 2025", "Arborbotics 2025");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      if (!replay) {
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      } else {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
      }
    }

    swerve.setDefaultCommand(
        swerve.driveTeleop(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.1)
                        * Constants.Swerve.maxLinearSpeed
                        * 0.8
                        * elevator.driveSpeedMultiplier()
                        * (slowMode ? 0.3 : 1),
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.1)
                        * Constants.Swerve.maxLinearSpeed
                        * 0.8
                        * elevator.driveSpeedMultiplier()
                        * (slowMode ? 0.3 : 1),
                    -MathUtil.applyDeadband(driver.getRightX(), 0.1)
                        * Constants.Swerve.maxAngularSpeed
                        * 0.5
                        * Math.sqrt(elevator.driveSpeedMultiplier())
                        * (slowMode ? 0.3 : 1)))); // the sqrt makes the multiplier less
    // strong for rotating

    debug
        .a()
        .onTrue(
            Commands.runOnce(
                () ->
                    swerve.resetPose(
                        new Pose2d(
                            swerve.getPose().getTranslation(),
                            (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
                                ? Rotation2d.k180deg
                                : Rotation2d.kZero))));
    debug
        .b()
        .onTrue(outtake.changeRollerSetpoint(-0.4))
        .onFalse(outtake.changeRollerSetpoint(-0.1));
    debug
        .x()
        .onTrue(
            Commands.runOnce(
                () -> swerve.resetPose(new Pose2d(4, 4, Rotation2d.kCCW_90deg)), swerve));
    driver
        .a()
        .whileTrue(
            Commands.sequence(
                superstructure.AutoAim(false),
                Commands.waitUntil(elevator::atSetpoint),
                // superstructure.Score(),
                Commands.parallel(
                    rumble(0.25, 1),
                    swerve.driveRR(
                        () ->
                            new ChassisSpeeds(
                                -MathUtil.applyDeadband(driver.getLeftY(), 0.1)
                                    * Constants.Swerve.maxLinearSpeed
                                    * 0.1,
                                -MathUtil.applyDeadband(driver.getLeftX(), 0.1)
                                    * Constants.Swerve.maxLinearSpeed
                                    * 0.1,
                                -MathUtil.applyDeadband(driver.getRightX(), 0.1)
                                    * Constants.Swerve.maxAngularSpeed
                                    * 0.1)))));

    driver
        .a()
        .onFalse(Commands.runOnce(() -> swerve.drive(new ChassisSpeeds())).andThen(rumble(0, 0)));

    driver
        .b()
        .whileTrue(
            new DriveToPose(
                    swerve,
                    superstructure::getNearestSubstation,
                    Units.inchesToMeters(1),
                    Units.degreesToRadians(1),
                    new Constraints(5, 3),
                    new Constraints(Units.rotationsToRadians(3), Units.rotationsToRadians(6)))
                .alongWith(superstructure.ElevatorIntake())
                .beforeStarting(superstructure.selectPiece("Coral")));

    driver
        .b()
        .onFalse(Commands.runOnce(() -> swerve.drive(new ChassisSpeeds())).andThen(rumble(0, 0)));

    driver.x().onTrue(Commands.runOnce(() -> this.slowMode = !this.slowMode));

    operator.button(9).onTrue(superstructure.selectElevatorHeight(2)); // 9
    operator.button(8).onTrue(superstructure.selectElevatorHeight(3)); // 8
    operator.button(7).onTrue(superstructure.selectElevatorHeight(4)); // 7

    operator.button(2).onTrue(superstructure.selectPiece("Coral")); // 2
    operator.button(5).onTrue(superstructure.selectPiece("Algae")); // 5

    operator.button(6).onTrue(superstructure.selectReef("Left")); // 6
    operator.button(3).onTrue(superstructure.selectReef("Right")); // 3

    operator.button(1).onTrue(superstructure.RaiseClimber()).onFalse(climber.setVoltage(0)); // 1
    operator.button(4).onTrue(superstructure.LowerClimber()).onFalse(climber.setVoltage(0)); // 4

    // Manual Elevator Controls
    driver.povUp().onTrue(superstructure.raiseElevator());
    driver.povDown().onTrue(elevator.changeSetpoint(0));

    // Ground Intake
    driver
        .rightTrigger()
        .onTrue(superstructure.GroundIntake())
        .onFalse(superstructure.RetractIntake());

    // HP Intake
    driver
        .rightBumper()
        .onTrue(superstructure.ElevatorIntake())
        .onFalse(superstructure.PassiveElevatorIntake());

    // General Score
    driver.leftTrigger().onTrue(superstructure.Score()).onFalse(superstructure.HomeRobot());

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    superstructure.update3DPose();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autoChooser.get().schedule();
  }

  @Override
  public void disabledInit() {
    intake.changePivotIdlemode(IdleMode.kCoast);
  }

  @Override
  public void disabledExit() {
    intake.changePivotIdlemode(IdleMode.kBrake);
  }

  @Override
  public void teleopInit() {
    if (autoChooser.get() != null) {
      autoChooser.get().cancel();
      superstructure.HomeRobot().schedule();
    }
  }

  private Command rumble(double duration, double intensity) {
    return Commands.sequence(
        Commands.runOnce(() -> driver.setRumble(RumbleType.kRightRumble, intensity)),
        Commands.waitSeconds(duration),
        Commands.runOnce(() -> driver.setRumble(RumbleType.kRightRumble, 0)));
  }
}
