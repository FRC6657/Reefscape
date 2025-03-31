package frc.robot.util;

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivebase.Swerve;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  private final Swerve drive;
  private final Supplier<Pose2d> target;

  private TrapezoidProfile driveProfile;
  private final PIDController driveController = new PIDController(3.5, 0.0, 0.0, 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(4, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Translation2d lastSetpointVelocity = Translation2d.kZero;
  private Rotation2d lastGoalRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private Supplier<Pose2d> robot;
  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  public DriveToPose(
      Swerve drive,
      Supplier<Pose2d> target,
      double tTol,
      double rTol,
      Constraints tCon,
      Constraints rCon) {
    this.drive = drive;
    this.target = target;

    robot = drive::getPose;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(tTol, tTol / 2);
    thetaController.setTolerance(rTol, rTol / 2);

    driveProfile = new TrapezoidProfile(tCon);
    thetaController.setConstraints(rCon);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();
    ChassisSpeeds fieldVelocity = drive.getVelocityFieldRelative();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset();
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointVelocity = linearFieldVelocity;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();
  }

  @Override
  public void execute() {

    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(Units.rotationsToRadians(1), Units.rotationsToRadians(1)));

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();

    Pose2d poseError = currentPose.relativeTo(targetPose);
    driveErrorAbs = poseError.getTranslation().getNorm();
    thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());
    double linearFFScaler = MathUtil.clamp((driveErrorAbs - 0.01) / (0.05 - 0.01), 0.0, 1.0);
    double thetaFFScaler =
        MathUtil.clamp(
            (Units.radiansToDegrees(thetaErrorAbs) - Units.degreesToRadians(2))
                / (Units.degreesToRadians(4) - Units.degreesToRadians(2)),
            0.0,
            1.0);

    // Calculate drive velocity
    // Calculate setpoint velocity towards target pose
    var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
    double setpointVelocity =
        direction.norm() <= 0.01 // Don't calculate velocity in direction when really close
            ? lastSetpointVelocity.getNorm()
            : lastSetpointVelocity.toVector().dot(direction) / direction.norm();
    setpointVelocity = Math.max(setpointVelocity, -0.5);
    State driveSetpoint =
        driveProfile.calculate(
            0.02,
            new State(
                direction.norm(), -setpointVelocity), // Use negative as profile has zero at target
            new State(0.0, 0.0));
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position)
            + driveSetpoint.velocity * linearFFScaler;
    if (driveErrorAbs < driveController.getErrorTolerance()) driveVelocityScalar = 0.0;
    Rotation2d targetToCurrentAngle =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
    Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(new Transform2d(driveSetpoint.position, 0.0, Rotation2d.kZero))
            .getTranslation();
    lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(
                    targetPose.getRotation().getRadians(),
                    (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                        / (Timer.getTimestamp() - lastTime)))
            + thetaController.getSetpoint().velocity * thetaFFScaler;
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();

    // Scale feedback velocities by input ff
    final double linearS = MathUtil.clamp(linearFF.get().getNorm() * 3.0, 0.0, 1.0);
    final double thetaS = MathUtil.clamp(Math.abs(omegaFF.getAsDouble()) * 3.0, 0.0, 1.0);
    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(Constants.Swerve.maxLinearSpeed), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * Constants.Swerve.maxAngularSpeed, thetaS);
    // Reset profiles if enough input
    ChassisSpeeds fieldVelocity = drive.getVelocityFieldRelative();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    if (linearS >= 0.2) {
      lastSetpointTranslation = currentPose.getTranslation();
      lastSetpointVelocity = linearFieldVelocity;
    }
    if (thetaS >= 0.1) {
      thetaController.reset(
          currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    }

    // Command speeds
    drive.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
    Logger.recordOutput(
        "DriveToPose/VelocityMeasured",
        -linearFieldVelocity
                .toVector()
                .dot(targetPose.getTranslation().minus(currentPose.getTranslation()).toVector())
            / driveErrorAbs);
    Logger.recordOutput("DriveToPose/VelocitySetpoint", driveSetpoint.velocity);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds());
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  @Override
  public boolean isFinished() {

    double driveTol = driveController.getErrorTolerance();
    double thetaTol = thetaController.getPositionTolerance();
    double thetaVelTol = thetaController.getVelocityTolerance();

    if (driveErrorAbs > driveTol || thetaErrorAbs > thetaTol) return false;
    if (Math.abs(driveController.getPositionError()) > driveTol) return false;
    if (Math.abs(thetaController.getPositionError()) > thetaTol) return false;
    if (Math.abs(thetaController.getVelocityError()) > thetaVelTol) return false;

    return true;
  }
}
