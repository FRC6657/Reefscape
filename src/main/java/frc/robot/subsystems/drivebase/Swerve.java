package frc.robot.subsystems.drivebase;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final Module[] modules; // FL FR BL BR

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  static final Lock odometryLock = new ReentrantLock();
  static final double odometryFrequency = 150;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

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
            VecBuilder.fill(2, 2, 1));

    this.gyroIO = gyroIO;
    this.modules = new Module[moduleIOs.length];

    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }

    // Enable Wrapping
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    PhoenixOdometryThread.getInstance().start();
  }

  /**
   * @return The current pose of the robot
   */
  @AutoLogOutput(key = "Swerve/Pose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @param pose The new pose of the robot
   */
  public void resetPose(Pose2d pose) {

    var yaw =
        (RobotBase.isSimulation() && !Robot.replay) ? pose.getRotation() : gyroInputs.yawPosition;
    poseEstimator.resetPosition(
        yaw,
        Arrays.stream(modules).map(m -> m.getPosition()).toArray(SwerveModulePosition[]::new),
        pose);
  }

  /**
   * @return The velocity of the robot in robot relative coordinates
   */
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
  public ChassisSpeeds getVelocityFieldRelative() {
    var speeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map(m -> m.getState()).toArray(SwerveModuleState[]::new));
    speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getPose().getRotation());
    return speeds;
  }

  public void drive(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  /**
   * Runs the drivetrain at the given robot relative speeds
   *
   * @param speeds The desired robot relative speeds
   */
  public void drive(ChassisSpeeds speeds, boolean openLoop) {

    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxLinearSpeed);
    final SwerveModuleState[] optimizedModuleStates = new SwerveModuleState[moduleStates.length];

    for (int i = 0; i < moduleStates.length; i++) {
      optimizedModuleStates[i] = modules[i].runSetpoint(moduleStates[i], openLoop); // Run setpoints
    }

    Logger.recordOutput("Swerve/ModuleSetpoints", optimizedModuleStates);
    Logger.recordOutput(
        "Swerve/ModuleStates",
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
          this.drive(speed, true);
        });
  }

  public Command driveRR(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          this.drive(speeds.get());
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

  ProfiledPIDController xController = AutoConstants.kXController_Position;
  ProfiledPIDController yController = AutoConstants.kYController_Position;
  ProfiledPIDController thetaController = AutoConstants.kThetaController_Position;

  /**
   * Reset the states of the auto aim PID controllers, this stops issues when jumping between
   * locations.
   */
  public Command resetAutoAimPID() {
    return Commands.runOnce(
        () -> {
          xController.reset(getPose().getX(), getVelocityFieldRelative().vxMetersPerSecond);
          yController.reset(getPose().getY(), getVelocityFieldRelative().vyMetersPerSecond);
          thetaController.reset(
              getPose().getRotation().getRadians(),
              getVelocityFieldRelative().omegaRadiansPerSecond);
        });
  }

  public BooleanSupplier atPoseFine(Supplier<Pose2d> targetPose) {
    return () ->
        atPose(
            targetPose.get(),
            Units.inchesToMeters(0.5), // Translation Tolerance Meters
            Units.degreesToRadians(2.0), // Rotation Tolerance Rad
            0.1, // Translation Velocity Tolerance m/s
            Units.rotationsToRadians(0.5) // Rotational Velocity Tolerance rad/s
            );
  }

  public BooleanSupplier atPoseCoarse(Supplier<Pose2d> targetPose) {
    return () ->
        atPose(
            targetPose.get(),
            Units.inchesToMeters(6.0), // Translation Tolerance Meters
            Units.degreesToRadians(3.0), // Rotation Tolerance Rad
            1, // Translation Velocity Tolerance m/s
            Units.rotationsToRadians(2.0) // Rotational Velocity Tolerance rad/s
            );
  }

  public boolean atPose(
      Pose2d targetPose,
      double translationTolerance,
      double rotationTolerance,
      double translationVelocityTolerance,
      double rotationVelocityTolerance) {

    var currentPose = getPose();

    double xError = Math.abs(currentPose.getX() - targetPose.getX());
    double yError = Math.abs(currentPose.getY() - targetPose.getY());
    double thetaError =
        Math.abs(currentPose.getRotation().getRadians())
            - Math.abs(targetPose.getRotation().getRadians());
    double xVelocity = Math.abs(getVelocityRobotRelative().vxMetersPerSecond);
    double yVelocity = Math.abs(getVelocityRobotRelative().vyMetersPerSecond);
    double thetaVelocity = Math.abs(getVelocityRobotRelative().omegaRadiansPerSecond);

    boolean atX = xError < translationTolerance;
    boolean atY = yError < translationTolerance;
    boolean atTheta = thetaError < rotationTolerance;
    boolean atXVelocity = xVelocity < translationVelocityTolerance;
    boolean atYVelocity = yVelocity < translationVelocityTolerance;
    boolean atThetaVelocity = thetaVelocity < rotationVelocityTolerance;

    Logger.recordOutput("AutoAim/T Tolerance", translationTolerance);
    Logger.recordOutput("AutoAim/R Tolerance", rotationTolerance);
    Logger.recordOutput("AutoAim/ V Tolerance", translationVelocityTolerance);
    Logger.recordOutput("AutoAim/R V Tolerance", rotationVelocityTolerance);
    Logger.recordOutput("AutoAim/X Error", xError);
    Logger.recordOutput("AutoAim/Y Error", yError);
    Logger.recordOutput("AutoAim/Theta Error", thetaError);
    Logger.recordOutput("AutoAim/AtX", atX);
    Logger.recordOutput("AutoAim/AtY", atY);
    Logger.recordOutput("AutoAim/AtTheta", atTheta);
    Logger.recordOutput("AutoAim/AtXVelocity", atXVelocity);
    Logger.recordOutput("AutoAim/AtYVelocity", atYVelocity);
    Logger.recordOutput("AutoAim/AtThetaVelocity", atThetaVelocity);

    return (xError < translationTolerance
        && yError < translationTolerance
        && thetaError < rotationTolerance
        && xVelocity < translationVelocityTolerance
        && yVelocity < translationVelocityTolerance
        && thetaVelocity < rotationVelocityTolerance);
  }

  /**
   * Default position controller with 0.5 inch translation tolerance and 2 degree rotation tolerance
   * 1 m/s translation and 2 rad/s rotation speed targets. Generally good for high precision
   * movement.
   */
  public void positionController(Pose2d targetPose) {
    positionController(
        targetPose,
        new Constraints(1, 2),
        new Constraints(Units.rotationsToRadians(1), Units.rotationsToRadians(2)));
  }

  /**
   * Runs the drivebase to a given pose with the given tolerances and constraints
   *
   * @param targetPose The target pose
   * @param translationTolerance The translation tolerance for stopping the movement
   * @param rotationTolerance The rotation tolerance for stopping the movement
   * @param translationConstraints The translation velocity/acceleration targets
   * @param rotationConstraints The rotation velocity/acceleration targets
   */
  public void positionController(
      Pose2d targetPose, Constraints translationConstraints, Constraints rotationConstraints) {

    xController.setConstraints(translationConstraints);
    yController.setConstraints(translationConstraints);
    thetaController.setConstraints(rotationConstraints);

    double xFF = xController.getSetpoint().velocity;
    double yFF = yController.getSetpoint().velocity;
    double rotationFF = thetaController.getSetpoint().velocity;

    double xFeedback = xController.calculate(getPose().getX(), targetPose.getX());
    double yFeedback = yController.calculate(getPose().getY(), targetPose.getY());
    double rotationFeedback =
        thetaController.calculate(
            getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

    Logger.recordOutput("Debug/X Setpoint", targetPose.getX());
    Logger.recordOutput("Debug/Y Setpoint", targetPose.getY());
    Logger.recordOutput("Debug/XOut", xFeedback + xFF);
    Logger.recordOutput("Debug/YOut", yFeedback + yFF);
    Logger.recordOutput("Debug/ROut", rotationFeedback + rotationFF);
    Logger.recordOutput("Debug/X FF", xFF);
    Logger.recordOutput("Debug/Y FF", yFF);
    Logger.recordOutput("Debug/R FF", rotationFF);

    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedback + xFF,
            yFeedback + yFF,
            rotationFeedback + rotationFF,
            getPose().getRotation());

    Logger.recordOutput("AutoAim/Target", targetPose);

    drive(out);
  }

  /**
   * Command to drive to a given pose with the given tolerances and constraints
   *
   * @param target The target pose
   * @param translationConstraints Translation velocity/acceleration targets
   * @param rotationConstraints Rotation velocity/acceleration targets
   * @return The command to drive to the given pose
   */
  public Command goToPoseFine(
      Supplier<Pose2d> target,
      Constraints translationConstraints,
      Constraints rotationConstraints) {
    return this.run(
            () -> positionController(target.get(), translationConstraints, rotationConstraints))
        .until(atPoseFine(target))
        .andThen(Commands.runOnce(() -> this.drive(new ChassisSpeeds())));
  }

  /**
   * Command to drive to a given pose with the given tolerances and constraints
   *
   * @param target The target pose
   * @param translationConstraints Translation velocity/acceleration targets
   * @param rotationConstraints Rotation velocity/acceleration targets
   * @return The command to drive to the given pose
   */
  public Command goToPoseCoarse(
      Supplier<Pose2d> target,
      Constraints translationConstraints,
      Constraints rotationConstraints) {
    return this.run(
            () -> positionController(target.get(), translationConstraints, rotationConstraints))
        .until(atPoseCoarse(target))
        .andThen(Commands.runOnce(() -> this.drive(new ChassisSpeeds())));
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (RobotBase.isReal() || Robot.replay) {
      poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }
  }

  /**
   * Runs a characterization routine that will determine the "real" wheel radius of the swerve
   * modules It does this by rotating the robot and measuring the distance traveled by the wheels.
   * It then compares the expected distance based on the gyro reading to the observed distance to
   * back calculate the "real" wheel radius.
   */
  public Command wheelRadiusCharacterization() {

    double driveRadius =
        Constants.Swerve.moduleTranslations[0].getNorm(); // Radius of drive wheel circle
    ChassisSpeeds speeds =
        new ChassisSpeeds(0, 0, Units.rotationsToRadians(0.2)); // Speed to rotate at

    return Commands.runOnce(
            () -> {
              // Reset Gyro and Zero Wheel Encoders
              gyroIO.setYaw(new Rotation2d());
              for (var module : modules) {
                module.resetDriveEncoder();
              }
            })
        .andThen(
            // Turn for 5 seconds at the desired speed to collect gyro/encoder data
            Commands.sequence(
                Commands.waitSeconds(1),
                Commands.runEnd(() -> this.drive(speeds), () -> this.drive(new ChassisSpeeds()))
                    .raceWith(Commands.waitSeconds(5))))
        .andThen(
            // Process collected data
            this.runOnce(
                () -> {

                  // Wheels should have all moved more or less the same distance, average them just
                  // incase.
                  double avgWheelRoations = 0;
                  for (var module : modules) {
                    avgWheelRoations +=
                        Math.abs(
                            module.getPosition().distanceMeters
                                / (Units.inchesToMeters(Constants.Swerve.wheelRadiusMeters)
                                    * 2
                                    * Math.PI));
                  }
                  avgWheelRoations /= 4;

                  // Compare the expected distance to the observed distance to back calculate the
                  // "real" wheel radius.
                  double expectedDistance =
                      (driveRadius * 2 * Math.PI) * Units.degreesToRotations(gyroInputs.yaw);
                  double observedRadius = expectedDistance / (avgWheelRoations * 2 * Math.PI);

                  // Log the calculated radius
                  Logger.recordOutput("Characterization/WheelRadius", observedRadius);
                }));
  }

  @Override
  public void periodic() {

    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (Module module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    if (RobotBase.isReal() || Robot.replay) {
      double[] sampleTimestamps = modules[0].getOdometryTimestamps();
      for (int i = 0; i < sampleTimestamps.length; i++) {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
          moduleDeltas[moduleIndex] =
              new SwerveModulePosition(
                  modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                  modulePositions[moduleIndex].angle);
          lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
          try {
            poseEstimator.updateWithTime(
                sampleTimestamps[i], gyroInputs.yawPositions[i], modulePositions);
          } catch (Exception e) {
            Logger.recordOutput("Errors", "Pose Estimator failed to update: " + e.getMessage());
          }
        }
      }
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
  }
}
