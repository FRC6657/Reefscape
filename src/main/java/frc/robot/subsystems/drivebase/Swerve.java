// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.units.measure.Angle;
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
import static edu.wpi.first.units.Units.*;

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

  double translationTolerance = Units.inchesToMeters(1);
  double rotationTolerance = Units.degreesToRadians(3);

  public Swerve(ModuleIO[] moduleIOs, GyroIO gyroIO) {

    String[] moduleNames = new String[] {"Front Left", "Front Right", "Back Left", "Back Right"};

    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(moduleIOs[i], moduleNames[i]);
    }

    this.gyroIO = gyroIO;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(translationTolerance);
    yController.setTolerance(translationTolerance);
    thetaController.setTolerance(rotationTolerance);
  }

  /** Used for only teleop */
  public Command drive(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    return Commands.run(
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          Rotation2d rotation =
              getPose().getRotation().plus(new Rotation2d(alliance == Alliance.Blue ? 0 : Math.PI));
          this.driveChassisSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), rotation));
        },
        this);
  }

  public void driveChassisSpeeds(ChassisSpeeds desiredSpeeds) {
    var newSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 1 / Constants.mainLoopFrequency);
    var states = kinematics.toSwerveModuleStates(newSpeeds);
    for (int i = 0; i < 4; i++) {
      states[i].optimize(getModulePositions()[i].angle);
      modules[i].changeState(states[i]);
    }

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
      
      //Reject poses tilted too far
      if(Math.abs(visionPose.getRotation().getMeasureX().in(Degrees)) > 10 || Math.abs(visionPose.getRotation().getMeasureY().in(Degrees)) > 10) {
        Logger.recordOutput("Errors/RejectedPoses", visionPose);
        return;
      }
      //Reject poses too far off the floor
      if(Math.abs(visionPose.getTranslation().getZ()) > Units.inchesToMeters(12)) {
        Logger.recordOutput("Errors/RejectedPoses", visionPose);
        return;
      }

      poseEstimator.addVisionMeasurement(visionPose.toPose2d(), timestamp, stdDevs);
    }
  }

  public boolean atPose(Pose2d desiredPose) {

    double xError = xController.getError();
    double yError = yController.getError();
    double thetaError = thetaController.getError();

    Logger.recordOutput("AutoAim/XError", xError);
    Logger.recordOutput("AutoAim/YError", yError);
    Logger.recordOutput("AutoAim/ThetaError", thetaError);

    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }

  public Command goToPose(Supplier<Pose2d> target) {
    return this.run(
            () -> {
              positionController(target.get());
            })
        .until(() -> atPose(target.get()))
        .andThen(Commands.runOnce(() -> this.driveChassisSpeeds(new ChassisSpeeds())));
  }

  PIDController xController = AutoConstants.kXController_Position;
  PIDController yController = AutoConstants.kYController_Position;
  PIDController thetaController = AutoConstants.kThetaController_Position;

  public void positionController(Pose2d targetPose) {

    double xFeedback = xController.calculate(getPose().getX(), targetPose.getX());
    double yFeedback = yController.calculate(getPose().getY(), targetPose.getY());
    double rotationFeedback =
        thetaController.calculate(
            getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            MathUtil.clamp(xFeedback, -1, 1),
            MathUtil.clamp(yFeedback, -1, 1),
            MathUtil.clamp(rotationFeedback, -1, 1),
            getPose().getRotation());

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

    Logger.recordOutput("Swerve/Auto/DesiredPose", sample.getPose());

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
