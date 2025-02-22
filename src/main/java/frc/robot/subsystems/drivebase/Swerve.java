// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  // Array for storing modules.
  private Module[] modules =
      new Module[] {
        new Module(new ModuleIO() {}, ""),
        new Module(new ModuleIO() {}, ""),
        new Module(new ModuleIO() {}, ""),
        new Module(new ModuleIO() {}, "")
      };

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private Rotation2d simHeading = new Rotation2d();

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.Swerve.ModulePositions);
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics, new Rotation2d(gyroInputs.yaw), getModulePositions(), new Pose2d());

  public Swerve(ModuleIO[] moduleIOs, GyroIO gyroIO) {

    String[] moduleNames = new String[] {"Front Left", "Front Right", "Back Left", "Back Right"};

    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(moduleIOs[i], moduleNames[i]);
    }

    this.gyroIO = gyroIO;
  }

  public Command drive(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {

    return Commands.run(
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          Rotation2d rotation = new Rotation2d(getPose().getRotation().getRadians());
          if (alliance == Alliance.Red) {
            fieldRelativeSpeeds.get().vxMetersPerSecond *= -1;
            fieldRelativeSpeeds.get().vyMetersPerSecond *= -1;
            rotation = rotation.rotateBy(Rotation2d.fromDegrees(180));
          }

          this.driveChassisSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), rotation));

          // this.driveChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getPose().getRotation()));
        },
        this);
  }

  @AutoLogOutput(key = "Swerve/FieldRelativeChassisSpeed")
  public ChassisSpeeds getCurrentFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(getModuleStates()), getPose().getRotation());
  }

  public void driveChassisSpeeds(ChassisSpeeds desiredSpeeds) {
    var newSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 1 / Constants.mainLoopFrequency);
    var states = kinematics.toSwerveModuleStates(newSpeeds);
    for (int i = 0; i < 4; i++) {
      states[i].optimize(getModulePositions()[i].angle);
      modules[i].changeState(states[i]);
    }

    Logger.recordOutput(
        "Swerve/Field Relative Chassis Speed Setpoint",
        ChassisSpeeds.fromRobotRelativeSpeeds(newSpeeds, getPose().getRotation()));
    Logger.recordOutput("Swerve/Setpoints", states);
  }

  public Command resetOdometry(Pose2d newPose) {
    return Commands.runOnce(
            () -> {
              var yaw =
                  RobotBase.isSimulation() ? newPose.getRotation() : new Rotation2d(gyroInputs.yaw);
              poseEstimator.resetPosition(yaw, getModulePositions(), newPose);
            })
        .andThen(Commands.print("Pose Reset"));
  }

  public void resetOdometryChoreo(Pose2d newPose) {
    resetOdometry(newPose).schedule();
  }

  @AutoLogOutput(key = "Swerve/Positions")
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      modules[0].getModulePosition(),
      modules[1].getModulePosition(),
      modules[2].getModulePosition(),
      modules[3].getModulePosition()
    };
  }

  @AutoLogOutput(key = "Swerve/States")
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      modules[0].getModuleState(),
      modules[1].getModuleState(),
      modules[2].getModuleState(),
      modules[3].getModuleState()
    };
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(Pose3d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (RobotBase.isReal()) {
      poseEstimator.addVisionMeasurement(getPose(), timestamp, stdDevs);
    }
  }

  public boolean atPose(Pose2d desiredPose) {
    double translationTolerance = Units.inchesToMeters(1);
    double rotationTolerance = Units.degreesToRadians(3);

    Pose2d currentPose = getPose();

    return currentPose.getTranslation().getDistance(desiredPose.getTranslation())
            < translationTolerance
        && Math.abs(currentPose.getRotation().getRadians() - desiredPose.getRotation().getRadians())
            < rotationTolerance;
  }

  public Command goToPose(Supplier<Pose2d> target) {
    return this.run(
            () -> {
              Logger.recordOutput("Swerve/PositioningMode", "PID");
              positionController(target.get());
            })
        .until(() -> atPose(target.get()))
        .andThen(Commands.runOnce(() -> this.driveChassisSpeeds(new ChassisSpeeds())));
  }

  public void positionController(Pose2d targetPose) {

    PIDController xController = AutoConstants.kXController_Position;
    PIDController yController = AutoConstants.kYController_Position;
    PIDController thetaController = AutoConstants.kThetaController_Position;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    double xFeedback = xController.calculate(getPose().getX(), targetPose.getX());
    double yFeedback = yController.calculate(getPose().getY(), targetPose.getY());
    double rotationFeedback =
        thetaController.calculate(
            getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedback, yFeedback, rotationFeedback, getPose().getRotation());

    Logger.recordOutput("AutoAim/AtSetpointX", xController.atSetpoint());
    Logger.recordOutput("AutoAim/AtSetpointY", yController.atSetpoint());
    Logger.recordOutput("AutoAim/AtSetpointTheta", thetaController.atSetpoint());

    driveChassisSpeeds(out);
  }

  PIDController choreoXController = AutoConstants.kXController_Choreo;
  PIDController choreoYController = AutoConstants.kYController_Choreo;
  PIDController choreoThetaController = AutoConstants.kThetaController_Choreo;

  public void followTrajectory(SwerveSample sample) {

    Pose2d currentPose = getPose();

    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    Logger.recordOutput("Swerve/Auto/DesiredPose", sample.getPose());
    Logger.recordOutput("Swerve/Auto/DesiredXVelocity", sample.vx);
    Logger.recordOutput("Swerve/Auto/DesiredYVelocity", sample.vx);

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

    driveChassisSpeeds(out);
  }

  public void periodic() {

    for (var module : modules) {
      module.updateInputs();
    }
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Gyro/", gyroInputs);

    if (RobotBase.isReal()) {
      poseEstimator.update(new Rotation2d(gyroInputs.yaw), getModulePositions());
    } else {
      // Simulate Gyro Heading
      simHeading = poseEstimator.getEstimatedPosition().getRotation();
      var gyroDelta =
          new Rotation2d(
              kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                  * (1 / Constants.mainLoopFrequency));
      simHeading = simHeading.plus(gyroDelta);
      poseEstimator.update(simHeading, getModulePositions());
    }

    Logger.recordOutput("Swerve/Pose", poseEstimator.getEstimatedPosition());
  }
}
