// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve.ModuleInformation;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO_Sim;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.example_intake.ExampleIntake;
import frc.robot.subsystems.vision.ApriltagCamera;
import frc.robot.subsystems.vision.ApriltagCameraIO_Real;
import frc.robot.subsystems.vision.ApriltagCameraIO_Sim;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private CommandXboxController driver = new CommandXboxController(0);

  private Swerve drivebase;
  private ExampleIntake intake;
  private ApriltagCamera[] cameras;

  private Superstructure superstructure;

  private final AutoFactory autoFactory;

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

    intake = new ExampleIntake();

    cameras =
        new ApriltagCamera[] {
          new ApriltagCamera(
              RobotBase.isReal()
                  ? new ApriltagCameraIO_Real(VisionConstants.cameraInfo)
                  : new ApriltagCameraIO_Sim(VisionConstants.cameraInfo),
              VisionConstants.cameraInfo)
        };

    superstructure = new Superstructure(drivebase, intake);

    autoFactory =
        new AutoFactory(
                drivebase::getPose,
                drivebase::resetOdometry,
                drivebase::followTrajectory,
                true,
                drivebase)
            .bind("intake_down", intake.down())
            .bind("intake_up", intake.up());
  }

  @SuppressWarnings("resource")
  @Override
  public void robotInit() {

    Logger.recordMetadata("ArborSwerveMK4i", "ArborSwerveMK4i");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }

    drivebase.setDefaultCommand(
        drivebase.drive(
            () ->
                new ChassisSpeeds(
                    MathUtil.applyDeadband(-driver.getLeftY(), 0.1) * 5,
                    MathUtil.applyDeadband(-driver.getLeftX(), 0.1) * 5,
                    MathUtil.applyDeadband(-driver.getRightX(), 0.1) * 7)));

    driver
        .a()
        .onTrue(
            drivebase.resetOdometry(
                new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(30))));

    driver.leftTrigger().onTrue(Commands.runOnce(intake::up, intake));
    driver.rightTrigger().onTrue(Commands.runOnce(intake::down, intake));

    driver.b().whileTrue(drivebase.repulsorCommand(() -> new Pose2d(2, 5.5, Rotation2d.kZero)));

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    for (var camera : cameras) {
      if (RobotBase.isSimulation()) {
        camera.updateSimPose(drivebase.getPose());
      }
      camera.updateInputs();
      drivebase.addVisionMeasurement(
          camera.getEstimatedPose(), camera.getLatestTimestamp(), camera.getLatestStdDevs());
    }

    superstructure.update3DPose();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    superstructure.testAuto(autoFactory).cmd().schedule();
  }
}
