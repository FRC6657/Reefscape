package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class Constants {

  public static double mainLoopFrequency = 50d; // Hz

  public static enum CAN {
    Swerve_FR_D(1),
    Swerve_FL_D(2),
    Swerve_BR_D(3),
    Swerve_BL_D(4),
    Swerve_FR_T(5),
    Swerve_FL_T(6),
    Swerve_BR_T(7),
    Swerve_BL_T(8),
    Swerve_FR_E(9),
    Swerve_FL_E(10),
    Swerve_BR_E(11),
    Swerve_BL_E(12),
    Gyro(13),
    OuttakeMotor(14),
    Elevetor_Leader(15),
    Elevator_Follower(16),
    AlgaeMotor(17),
    IntakePivot(19),
    IntakeRoller(20),
    IntakeEncoder(21),
    Climber(22);

    public int id;

    CAN(int id) {
      this.id = id;
    }
  }

  public static class FieldConstants {

    private static Pose2d getRedReefPose(Pose2d reefPose) {
      return new Pose2d(
          reefPose.getTranslation().getX() + 8.565,
          reefPose.getTranslation().getY(),
          reefPose.getRotation());
    }

    public static class ReefSlot {
      public Pose2d middle;
      public Pose2d left;
      public Pose2d right;
      public Pose2d algae;

      ReefSlot(Pose2d middle, Pose2d left, Pose2d right, Pose2d algae) {
        this.middle = middle;
        this.left = left;
        this.right = right;
        this.algae = algae;
      }
    }

    public static enum ReefPoses {
      Reef_1(new Pose2d(5.825, 4.03, Rotation2d.fromDegrees(0))),
      Reef_2(new Pose2d(5.163, 5.177484, Rotation2d.fromDegrees(60))),
      Reef_3(new Pose2d(3.838, 5.177484, Rotation2d.fromDegrees(120))),
      Reef_4(new Pose2d(3.175, 4.03, Rotation2d.fromDegrees(180))),
      Reef_5(new Pose2d(3.8375, 2.882516, Rotation2d.fromDegrees(-120))),
      Reef_6(new Pose2d(5.1625, 2.882516, Rotation2d.fromDegrees(-60)));

      public ReefSlot blue;
      public ReefSlot red;

      // Shift the pose to the robot's left
      public Pose2d getLeftPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(0.25, -0.26, new Rotation2d()));
      }

      public Pose2d getRightPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(0.25, 0.06, new Rotation2d()));
      }

      public Pose2d getAlgaePose(Pose2d pose) {
        return pose.transformBy(new Transform2d(0.25, -0.09, new Rotation2d()));
      }

      ReefPoses(Pose2d pose) {
        this.blue = new ReefSlot(pose, getLeftPose(pose), getRightPose(pose), getAlgaePose(pose));
        this.red =
            new ReefSlot(
                getRedReefPose(pose),
                getRedReefPose(getLeftPose(pose)),
                getRedReefPose(getRightPose(pose)),
                getRedReefPose(getAlgaePose(pose)));
      }
    }
  }

  public static class AutoConstants {

    // Choreo
    public static final PIDController kXController_Choreo = new PIDController(0.1, 0, 0);
    public static final PIDController kYController_Choreo = new PIDController(0.1, 0, 0);
    public static final PIDController kThetaController_Choreo = new PIDController(0.1, 0, 0);

    // Position PID

    public static final Constraints kAutoAimTranslationConstraints = new Constraints(2, 5);
    public static final Constraints kAutoAimRotationConstraints =
        new Constraints(Units.rotationsToRadians(3), Units.rotationsToRadians(10));

    public static final ProfiledPIDController kXController_Position =
        new ProfiledPIDController(3.5, 0, 0, kAutoAimTranslationConstraints);
    public static final ProfiledPIDController kYController_Position =
        new ProfiledPIDController(3.5, 0, 0, kAutoAimTranslationConstraints);
    public static final ProfiledPIDController kThetaController_Position =
        new ProfiledPIDController(3, 0, 0, kAutoAimRotationConstraints);
  }

  public static class VisionConstants {

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 1; // Meters
    public static double angularStdDevBaseline = 0.12; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final List<Integer> kRedReefTagIDs = List.of(6, 7, 8, 9, 10, 11);

    public static final List<Integer> kBlueReefTagIDs = List.of(17, 18, 19, 20, 21, 22);

    private static final List<AprilTag> kBlueReefTags =
        List.of(
            kTagLayout.getTags().get(16),
            kTagLayout.getTags().get(17),
            kTagLayout.getTags().get(18),
            kTagLayout.getTags().get(19),
            kTagLayout.getTags().get(20),
            kTagLayout.getTags().get(21));

    private static final List<AprilTag> kRedReefTags =
        List.of(
            kTagLayout.getTags().get(5),
            kTagLayout.getTags().get(6),
            kTagLayout.getTags().get(7),
            kTagLayout.getTags().get(8),
            kTagLayout.getTags().get(9),
            kTagLayout.getTags().get(10));

    private static final List<AprilTag> kReefTags =
        List.of(
            kTagLayout.getTags().get(5),
            kTagLayout.getTags().get(6),
            kTagLayout.getTags().get(7),
            kTagLayout.getTags().get(8),
            kTagLayout.getTags().get(9),
            kTagLayout.getTags().get(10),
            kTagLayout.getTags().get(16),
            kTagLayout.getTags().get(17),
            kTagLayout.getTags().get(18),
            kTagLayout.getTags().get(19),
            kTagLayout.getTags().get(20),
            kTagLayout.getTags().get(21));

    public static final AprilTagFieldLayout kBlueTagLayout =
        new AprilTagFieldLayout(
            kBlueReefTags, kTagLayout.getFieldLength(), kTagLayout.getFieldWidth());
    public static final AprilTagFieldLayout kRedTagLayout =
        new AprilTagFieldLayout(
            kRedReefTags, kTagLayout.getFieldLength(), kTagLayout.getFieldWidth());
    public static final AprilTagFieldLayout kReefTagLayout =
        new AprilTagFieldLayout(kReefTags, kTagLayout.getFieldLength(), kTagLayout.getFieldWidth());

    public static class CameraInfo {

      public String cameraName;
      public Transform3d robotToCamera;
      public Rotation2d diagFOV;
      public int[] cameraRes;

      public CameraInfo(
          String cameraName, Transform3d robotToCamera, Rotation2d diagFOV, int[] cameraRes) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
        this.diagFOV = diagFOV;
        this.cameraRes = cameraRes;
      }
    }

    public static CameraInfo BlackReefInfo =
        new CameraInfo(
            "Black_Reef",
            new Transform3d(
                new Translation3d(-0.320048, -0.300306, 0.299816),
                new Rotation3d(0, Units.degreesToRadians(0), Math.PI - Units.degreesToRadians(55))),
            Rotation2d.fromDegrees(79.19),
            new int[] {1280, 800});

    public static CameraInfo WhiteReefInfo =
        new CameraInfo(
            "White_Reef",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(3), Units.inchesToMeters(4), Units.inchesToMeters(9)),
                new Rotation3d(
                    Math.PI, Units.degreesToRadians(0), Math.PI + Units.degreesToRadians(20))),
            Rotation2d.fromDegrees(79.19),
            new int[] {1280, 800});
  }

  public static class Swerve {

    public record ModuleConstants(int id, String prefix, int driveID, int turnID, int encoderID) {}

    public static final double trackWidthX = Units.inchesToMeters(23.75);
    public static final double trackWidthY = Units.inchesToMeters(23.75);

    public static final double maxLinearSpeed = Units.feetToMeters(17.3);
    public static final double maxLinearAcceleration = 8;
    public static final double maxAngularSpeed =
        maxLinearSpeed / Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);

    public static final ModuleConstants frontLeftModule =
        new ModuleConstants(
            0, "Front Left", CAN.Swerve_FL_D.id, CAN.Swerve_FL_T.id, CAN.Swerve_FL_E.id);
    public static final ModuleConstants frontRightModule =
        new ModuleConstants(
            0, "Front Right", CAN.Swerve_FR_D.id, CAN.Swerve_FR_T.id, CAN.Swerve_FR_E.id);
    public static final ModuleConstants backLeftModule =
        new ModuleConstants(
            0, "Back Left", CAN.Swerve_BL_D.id, CAN.Swerve_BL_T.id, CAN.Swerve_BL_E.id);
    public static final ModuleConstants backRightModule =
        new ModuleConstants(
            0, "Back Right", CAN.Swerve_BR_D.id, CAN.Swerve_BR_T.id, CAN.Swerve_BR_E.id);
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double driveRatio = (45.0 / 15.0) * (16.0 / 28.0) * (50.0 / 14.0);
    public static final double turnRatio = (150.0 / 7.0);
    public static final double driveRotorToMeters = driveRatio / (wheelRadiusMeters * 2 * Math.PI);

    public static final Translation2d[] moduleTranslations = {
      new Translation2d(trackWidthX / 2, trackWidthY / 2),
      new Translation2d(trackWidthX / 2, -trackWidthY / 2),
      new Translation2d(-trackWidthX / 2, trackWidthY / 2),
      new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
    };

    public static TalonFXConfiguration driveConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(driveRotorToMeters))
            .withSlot0(new Slot0Configs().withKV(12d / maxLinearSpeed).withKS(0).withKP(2.25));
    // .withMotionMagic(
    //     new MotionMagicConfigs()
    //         .withMotionMagicCruiseVelocity(maxLinearSpeed)
    //         .withMotionMagicAcceleration(maxLinearAcceleration));

    public static TalonFXConfiguration turnConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(20)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(40)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(turnRatio))
            .withSlot0(
                new Slot0Configs()
                    .withKV(12d / ((6300d / 60) / turnRatio))
                    .withKS(0.27)
                    .withKP(25)
                    .withKD(0.6))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity((6300 / 60) / turnRatio)
                    .withMotionMagicAcceleration((6300 / 60) / (turnRatio * 0.005)))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));
  }

  public static class Intake {

    public static double pivotGearing = (60d / 1) * (72d / 28);
    public static double maxAngle = Units.degreesToRadians(116);
    public static double minAngle = Units.degreesToRadians(5);

    public static double coralScoreAngle = Units.degreesToRadians(15);
    public static double algaeScoreAngle = Units.degreesToRadians(5);

    public static final double kPivotSupplyLimit = 40;
    public static final double kRollersCurrentLimit = 30;

    public static final double kGroundIntakeSpeed = 0.7;
    public static final double kFeedSpeed = 0.25;

    public static final double pivotAtSetpointTolerance = 2.0; // degrees

    public static final CurrentLimitsConfigs kRollersCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kRollersCurrentLimit)
            .withSupplyCurrentLimit(kRollersCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerLimit(kRollersCurrentLimit)
            .withSupplyCurrentLowerTime(0);
  }

  public static class Outtake {
    public static double gearing = 11.0 / 24.0;

    public static final double kSupplyLimit = 20;
    public static final double kStatorLimit = 40;

    public static final CurrentLimitsConfigs currentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kStatorLimit)
            .withSupplyCurrentLimit(kSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerLimit(kSupplyLimit)
            .withSupplyCurrentLowerTime(0);
  }

  public static class Elevator {
    public static double gearing = (5d / 1) * (66d / 22);
    public static double sprocketPD = 0.25 / (Math.sin(Math.PI / 22)); // Inches
    public static double maxHeight = Units.inchesToMeters(60); // Carriage Travel (Meters)
    public static double minHeight = Units.inchesToMeters(0); // Carriage Travel (Meters)
    public static int stages = 3;
    public static double setpointTollerance = Units.inchesToMeters(1);

    public static double kAlgaeStrength = 0.4;

    public static Slot0Configs motorSlot0 =
        new Slot0Configs()
            .withKS(0) // Volts
            .withKG(0.35) // Volts
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(9.5)
            .withKI(0)
            .withKD(0);

    public static final double kSupplyLimit = 40;
    public static final double kStatorLimit = 60;

    public static final CurrentLimitsConfigs currentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kStatorLimit)
            .withSupplyCurrentLimit(kSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerLimit(kSupplyLimit)
            .withSupplyCurrentLowerTime(0);

    public static double kMaxVelocity = 114; // Inches/s of Carriage Travel
    public static double kMaxAcceleration = 1500; // Inches/s/s of Carriage Travel

    public static MotionMagicConfigs kMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity((kMaxVelocity / stages) / (sprocketPD * Math.PI))
            .withMotionMagicAcceleration((kMaxAcceleration / stages) / (sprocketPD * Math.PI));
  }

  public static class De_algaefier {
    public static double pivotGearing = (10d * 9) * (3d / 2);

    public static double maxAngle = Units.degreesToRotations(65);
    public static double minAngle = Units.degreesToRotations(0);
    // TODO find these angles

    public static final int kPivotSupplyLimit = 20;
  }

  public static final class ClimberConstants {
    public static double maxRotations = 90;
    public static double minRotations = 0;
    public static double gearing = 60d;
    public static int currentLimit = 40;
  }
}
