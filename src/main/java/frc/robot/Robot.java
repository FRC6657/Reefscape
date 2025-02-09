// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Swerve.ModuleInformation;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Superstructure;
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
import frc.robot.subsystems.vision.ApriltagCamera;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandGenericHID operator = new CommandGenericHID(1);

  private Swerve drivebase;

  private Elevator elevator;
  private Outtake outtake;
  private Intake intake;

  private ApriltagCamera[] cameras;

  private Superstructure superstructure;

  private final AutoFactory autoFactory;

  private LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  public Robot() {

    drivebase =
        new Swerve(
            RobotBase.isReal()
                ? new ModuleIO[] {
                  new ModuleIO_Real(ModuleInformation.frontLeft),
                  new ModuleIO_Real(ModuleInformation.frontRight),
                  new ModuleIO_Real(ModuleInformation.backLeft),
                  new ModuleIO_Real(ModuleInformation.backRight)
                }
                : new ModuleIO[] {
                  new ModuleIO_Sim(ModuleInformation.frontLeft),
                  new ModuleIO_Sim(ModuleInformation.frontRight),
                  new ModuleIO_Sim(ModuleInformation.backLeft),
                  new ModuleIO_Sim(ModuleInformation.backRight)
                },
            RobotBase.isReal() ? new GyroIO_Real() : new GyroIO() {});

    intake = new Intake(RobotBase.isReal() ? new IntakeIO_Real() : new IntakeIO_Sim());
    elevator = new Elevator(RobotBase.isReal() ? new ElevatorIO_Real() : new ElevatorIO_Sim());
    outtake = new Outtake(RobotBase.isReal() ? new OuttakeIO_Real() : new OuttakeIO_Sim());

    // cameras =
    //     new ApriltagCamera[] {
    //       new ApriltagCamera(
    //           RobotBase.isReal()
    //               ? new ApriltagCameraIO_Real(VisionConstants.camera1Info)
    //               : new ApriltagCameraIO_Sim(VisionConstants.camera1Info),
    //           VisionConstants.camera1Info),
    //       new ApriltagCamera(
    //           RobotBase.isReal()
    //               ? new ApriltagCameraIO_Real(VisionConstants.camera2Info)
    //               : new ApriltagCameraIO_Sim(VisionConstants.camera2Info),
    //           VisionConstants.camera2Info),
    //       new ApriltagCamera(
    //           RobotBase.isReal()
    //               ? new ApriltagCameraIO_Real(VisionConstants.camera3Info)
    //               : new ApriltagCameraIO_Sim(VisionConstants.camera3Info),
    //           VisionConstants.camera3Info)
    //     };

    superstructure = new Superstructure(drivebase, elevator, outtake, intake);

    autoFactory =
        new AutoFactory(
            drivebase::getPose,
            drivebase::resetOdometryChoreo,
            drivebase::followTrajectory,
            true,
            drivebase);

    autoChooser.addDefaultOption("None", Commands.print("No Auto Selected"));
    autoChooser.addOption("Test Auto", superstructure.testAuto(autoFactory, false).cmd());
    autoChooser.addOption("Test Auto Mirror", superstructure.testAuto(autoFactory, true).cmd());
  }

  @SuppressWarnings("resource")
  @Override
  public void robotInit() {

    Logger.recordMetadata("Arborbotics 2025", "Arborbotics 2025");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }

    Trigger coralDetected = new Trigger(outtake::coralDetected);

    drivebase.setDefaultCommand(
        drivebase.drive(
            () ->
                new ChassisSpeeds(
                    MathUtil.applyDeadband(-driver.getLeftY(), 0.1) * 2,
                    MathUtil.applyDeadband(-driver.getLeftX(), 0.1) * 2,
                    MathUtil.applyDeadband(-driver.getRightX(), 0.1) * 2)));

    driver.rightTrigger().onTrue(outtake.changeRollerSetpoint(-0.4));
    coralDetected
        .onTrue(outtake.changeRollerSetpoint(0))
        .onFalse(Commands.waitSeconds(0.3).andThen(outtake.changeRollerSetpoint(0)));
    driver.rightTrigger().onFalse(outtake.changeRollerSetpoint(0));

    // driver
    //     .a()
    //     .whileTrue(
    //         drivebase
    //             .goToPose(() -> superstructure.getNearestReef())
    //             .andThen(
    //                 Commands.sequence(
    //                     Commands.runOnce(() -> driver.setRumble(RumbleType.kRightRumble, 1)),
    //                     Commands.waitSeconds(0.25),
    //                     Commands.runOnce(() -> driver.setRumble(RumbleType.kRightRumble, 0)))));

    driver.povLeft().onTrue(superstructure.selectReef("Left"));
    driver.povRight().onTrue(superstructure.selectReef("Right"));

    operator.button(9).onTrue(superstructure.selectElevatorHeight(2)); // button 2
    operator.button(8).onTrue(superstructure.selectElevatorHeight(3)); // button 3
    operator.button(7).onTrue(superstructure.selectElevatorHeight(4)); // button 4

    driver
        .y()
        .onTrue(
            drivebase.resetOdometry(
                new Pose2d(
                    drivebase.getPose().getX(), drivebase.getPose().getY(), new Rotation2d())));

    driver.rightBumper().onTrue(superstructure.raiseElevator()).onFalse(elevator.changeSetpoint(0));

    driver // Coral Pickup
        .leftTrigger()
        .onTrue(
            Commands.sequence(
                intake.changePivotSetpoint(Units.degreesToRadians(2)),
                intake.changeRollerSpeed(-Constants.Intake.kGroundIntakeSpeed)))
        .onFalse(
            Commands.sequence(
                intake.changePivotSetpoint(Constants.Intake.maxAngle),
                intake.changeRollerSpeed(-Constants.Intake.kFeedSpeed / 1.5)));

    driver // Algae Pickup
        .leftBumper()
        .onTrue(
            Commands.sequence(
                intake.changePivotSetpoint(Units.degreesToRadians(60)),
                intake.changeRollerSpeed(Constants.Intake.kGroundIntakeSpeed)))
        .onFalse(
            Commands.sequence(
                intake.changePivotSetpoint(Constants.Intake.maxAngle),
                intake.changeRollerSpeed(Constants.Intake.kFeedSpeed)));

    // driver
    //     .a()
    //     .onTrue(intake.changeRollerSpeed(-Constants.Intake.kGroundIntakeSpeed))
    //     .onFalse(
    //         Commands.sequence(
    //             intake.changePivotSetpoint(Constants.Intake.maxAngle),
    //             intake.changeRollerSpeed(0)));

    operator.button(1).onTrue(intake.changeRollerSpeed(0));

    Logger.start();
  }

  @Override
  public void robotPeriodic() {

    // for (var camera : cameras) {
    //   if (RobotBase.isSimulation()) {
    //     camera.updateSimPose(drivebase.getPose());
    //   }
    //   camera.updateInputs();
    //   drivebase.addVisionMeasurement(
    //       camera.getEstimatedPose(), camera.getLatestTimestamp(), camera.getLatestStdDevs());
    // }

    superstructure.update3DPose();

    Logger.recordOutput(
        "ReefCam Pose1",
        new Pose3d(drivebase.getPose()).transformBy(VisionConstants.camera1Info.robotToCamera));

    Logger.recordOutput(
        "ReefCam Pose2",
        new Pose3d(drivebase.getPose()).transformBy(VisionConstants.camera2Info.robotToCamera));
    Logger.recordOutput(
        "ReefCam Pose3",
        new Pose3d(drivebase.getPose()).transformBy(VisionConstants.camera3Info.robotToCamera));

    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    intake.changePivotIdlemode(IdleMode.kCoast);
  }

  @Override
  public void disabledExit() {
    intake.changePivotIdlemode(IdleMode.kBrake);
  }

  @Override
  public void autonomousInit() {
    autoChooser.get().schedule();
  }
}
