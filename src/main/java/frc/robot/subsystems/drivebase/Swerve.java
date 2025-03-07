// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.ApriltagCamera;
import frc.robot.subsystems.vision.ApriltagCameraIO_Real;
import frc.robot.subsystems.vision.ApriltagCameraIO_Sim;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final Module[] modules; // FL FR BL BR

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final ApriltagCamera[] cameras;

  public Swerve(GyroIO gyroIO, ModuleIO[] moduleIOs) {

    this.kinematics = new SwerveDriveKinematics(Constants.Swerve.moduleTranslations);
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.6, 0.6, 0.07),
            VecBuilder.fill(0.9, 0.9, 0.4));

    this.gyroIO = gyroIO;
    this.modules = new Module[moduleIOs.length];

    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }

    cameras =
        new ApriltagCamera[] {
          new ApriltagCamera(
              RobotBase.isReal()
                  ? new ApriltagCameraIO_Real(VisionConstants.camera1Info)
                  : new ApriltagCameraIO_Sim(VisionConstants.camera1Info),
              VisionConstants.camera1Info),
          new ApriltagCamera(
              RobotBase.isReal()
                  ? new ApriltagCameraIO_Real(VisionConstants.camera2Info)
                  : new ApriltagCameraIO_Sim(VisionConstants.camera2Info),
              VisionConstants.camera2Info),
          // new ApriltagCamera(
          //     RobotBase.isReal()
          //         ? new ApriltagCameraIO_Real(VisionConstants.camera3Info)
          //         : new ApriltagCameraIO_Sim(VisionConstants.camera3Info),
          //     VisionConstants.camera3Info)
        };

    xController.setTolerance(Units.inchesToMeters(0.5));
    yController.setTolerance(Units.inchesToMeters(0.5));
    thetaController.setTolerance(Units.degreesToRadians(2.0));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * @return The current pose of the robot
   */
  @AutoLogOutput(key = "Odometry/Pose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @param pose The new pose of the robot
   */
  public void resetPose(Pose2d pose) {
    var yaw = RobotBase.isSimulation() ? pose.getRotation() : gyroInputs.yawPosition;
    poseEstimator.resetPosition(
        yaw,
        Arrays.stream(modules).map(m -> m.getPosition()).toArray(SwerveModulePosition[]::new),
        pose);
  }

  /**
   * @return The velocity of the robot in robot relative coordinates
   */
  @AutoLogOutput(key = "Odometry/Velocity Robot Relative")
  public ChassisSpeeds getVelocityRobotRelative() {
    var speeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new));
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /**
   * @return The velocity of the robot in field relative coordinates
   */
  @AutoLogOutput(key = "Odometry/Velocity Field Relative")
  public ChassisSpeeds getVelocityFieldRelative() {
    var speeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map(m -> m.getState()).toArray(SwerveModuleState[]::new));
    speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getPose().getRotation());
    return speeds;
  }

  /**
   * Runs the drivetrain at the given robot relative speeds
   *
   * @param speeds The desired robot relative speeds
   */
  public void drive(ChassisSpeeds speeds) {

    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxLinearSpeed);
    final SwerveModuleState[] optimizedModuleStates = new SwerveModuleState[moduleStates.length];

    for (int i = 0; i < moduleStates.length; i++) {
      optimizedModuleStates[i] = modules[i].runSetpoint(moduleStates[i]); // Run setpoints
    }

    Logger.recordOutput("SwerveStates/RawSetpoints", moduleStates);
    Logger.recordOutput("Swerve/RR Target Speeds", speeds);
    Logger.recordOutput("Swerve/RR Speed Error", speeds.minus(getVelocityRobotRelative()));
    Logger.recordOutput(
        "Swerve/FR Target Speeds",
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));
    Logger.recordOutput(
        "Swerve/FR Speed Error",
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation())
            .minus(getVelocityFieldRelative()));
    Logger.recordOutput("SwerveStates/OptimizedSetpoints", optimizedModuleStates);
    Logger.recordOutput(
        "SwerveStates/ObservedStates",
        Arrays.stream(modules).map(m -> m.getState()).toArray(SwerveModuleState[]::new));
  }

  /**
   * Runs the drivetrain at the given robot relative speeds
   *
   * @param speeds The desired field relative speeds
   * @return The command to run the drivetrain at the given robot relative speeds
   */
  public Command driveVelocity(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(speeds.get()));
  }

  /**
   * Runs the drivetrain at the given field relative speeds
   *
   * @param speeds The desired field relative speeds
   * @return The command to run the drivetrain at the given field relative speeds
   */
  public Command driveVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.driveVelocity(
        () -> {
          var speed = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getPose().getRotation());
          return speed;
        });
  }

  /**
   * Runs the drivetrain for teleop with field relative speeds
   *
   * @param speeds The desired field relative speeds
   * @return The command to run the drivetrain for teleop with field relative speeds
   */
  public Command driveTeleop(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          var speed =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds.get(),
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? getPose().getRotation()
                      : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
          this.drive(speed);
        });
  }

  PIDController choreoXController = AutoConstants.kXController_Choreo;
  PIDController choreoYController = AutoConstants.kYController_Choreo;
  PIDController choreoThetaController = AutoConstants.kThetaController_Choreo;

  public void followTrajectory(SwerveSample sample) {

    Pose2d currentPose = getPose();

    Logger.recordOutput("Choreo/DesiredPose", sample.getPose());

    double xFF = sample.vx;
    double yFF = sample.vy;
    double rotationFF = sample.omega;

    double xFeedback = choreoXController.calculate(currentPose.getX(), sample.x);
    double yFeedback = choreoYController.calculate(currentPose.getY(), sample.y);
    double rotationFeedback =
        choreoThetaController.calculate(currentPose.getRotation().getRadians(), sample.heading);

    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            currentPose.getRotation());

    drive(out);
  }

  PIDController xController = AutoConstants.kXController_Position;
  PIDController yController = AutoConstants.kYController_Position;
  PIDController thetaController = AutoConstants.kThetaController_Position;

  public void positionController(
      Pose2d targetPose, double translationTolerance, double rotationTolerance, double speed) {

    xController.setTolerance(translationTolerance, Units.inchesToMeters(0.125));
    yController.setTolerance(translationTolerance, Units.inchesToMeters(0.125));
    thetaController.setTolerance(rotationTolerance, Units.degreesToRadians(1));

    double xFeedback = xController.calculate(getPose().getX(), targetPose.getX());
    double yFeedback = yController.calculate(getPose().getY(), targetPose.getY());
    double rotationFeedback =
        thetaController.calculate(
            getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

    double translationClamp = Constants.Swerve.maxLinearSpeed;
    double rotationClamp = Constants.Swerve.maxAngularSpeed;

    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            MathUtil.clamp(xFeedback, -translationClamp * speed, translationClamp * speed),
            MathUtil.clamp(yFeedback, -translationClamp * speed, translationClamp * speed),
            MathUtil.clamp(rotationFeedback, -rotationClamp * speed, rotationClamp * speed),
            getPose().getRotation());

    Logger.recordOutput("AutoAim/Target", targetPose);
    Logger.recordOutput("AutoAim/AtSetpointX", xController.atSetpoint());
    Logger.recordOutput("AutoAim/AtSetpointY", yController.atSetpoint());
    Logger.recordOutput("AutoAim/AtSetpointTheta", thetaController.atSetpoint());

    drive(out);
  }

  public Command goToPose(
      Supplier<Pose2d> target,
      double translationTolerance,
      double rotationTolerance,
      double speed) {
    return this.run(
            () -> positionController(target.get(), translationTolerance, rotationTolerance, speed))
        .until(
            () -> {
              return xController.atSetpoint()
                  && yController.atSetpoint()
                  && thetaController.atSetpoint();
            })
        .andThen(Commands.runOnce(() -> this.drive(new ChassisSpeeds())));
  }

  public void addVisionMeasurement(Pose3d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (RobotBase.isReal()) {

      if (Math.abs(visionPose.getRotation().getMeasureX().in(Degrees)) > 10
          || Math.abs(visionPose.getRotation().getMeasureY().in(Degrees)) > 10) {
        Logger.recordOutput("Errors/RejectedPoses", visionPose);
        return;
      }
      // Reject poses too far off the floor
      if (Math.abs(visionPose.getTranslation().getZ()) > Units.inchesToMeters(12)) {
        Logger.recordOutput("Errors/RejectedPoses", visionPose);
        return;
      }

      poseEstimator.addVisionMeasurement(visionPose.toPose2d(), timestamp, stdDevs);
    }
  }

  @Override
  public void periodic() {

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (Module module : modules) {
      module.updateInputs();
    }

    if (RobotBase.isReal()) {
      poseEstimator.update(
          gyroInputs.yawPosition,
          Arrays.stream(modules).map(m -> m.getPosition()).toArray(SwerveModulePosition[]::new));
    } else {
      var simHeading = getPose().getRotation();
      var gyroDelta =
          new Rotation2d(
                  kinematics.toChassisSpeeds(
                          Arrays.stream(modules)
                              .map(m -> m.getState())
                              .toArray(SwerveModuleState[]::new))
                      .omegaRadiansPerSecond)
              .times(0.02);

      simHeading = simHeading.plus(gyroDelta);
      poseEstimator.update(
          simHeading,
          Arrays.stream(modules).map(m -> m.getPosition()).toArray(SwerveModulePosition[]::new));
    }

    for (var camera : cameras) {
      if (RobotBase.isSimulation()) {
        camera.updateSimPose(getPose());
      }
      camera.updateInputs(
          RobotBase.isSimulation() ? Timer.getFPGATimestamp() : gyroInputs.timestamp,
          RobotBase.isSimulation() ? getPose().getRotation() : gyroInputs.yawPosition);

      Pose3d estPose = camera.getEstimatedPose();
      if(estPose != new Pose3d(new Translation3d(100, 100, 100), new Rotation3d())){
        Logger.recordOutput("Vision/ProcessedPoses", estPose);
        addVisionMeasurement(estPose, camera.getLatestTimestamp(), camera.getLatestStdDevs());
      }
    }
  }
}
