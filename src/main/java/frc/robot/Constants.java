package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static double mainLoopFrequency = 50d; // Hz

  public static enum CAN {
    Swerve_FL_D(1),
    Swerve_FR_D(2),
    Swerve_BL_D(3),
    Swerve_BR_D(4),
    Swerve_FL_T(5),
    Swerve_FR_T(6),
    Swerve_BL_T(7),
    Swerve_BR_T(8),
    Swerve_FL_E(9),
    Swerve_FR_E(10),
    Swerve_BL_E(11),
    Swerve_BR_E(12),
    Gyro(13),
    OuttakeMotor(14),
    Elevetor_Leader(15),
    Elevator_Follower(16),
    IntakePivot(19),
    IntakeRoller(20),
    IntakeEncoder(21),
    Climber(22),
    AlgaeMotor(23);

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

      ReefSlot(Pose2d middle, Pose2d left, Pose2d right) {
        this.middle = middle;
        this.left = left;
        this.right = right;
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
        return pose.transformBy(new Transform2d(0, -0.26, new Rotation2d()));
      }

      public Pose2d getRightPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(0, 0.06, new Rotation2d()));
      }

      ReefPoses(Pose2d pose) {
        this.blue = new ReefSlot(pose, getLeftPose(pose), getRightPose(pose));
        this.red =
            new ReefSlot(
                getRedReefPose(pose),
                getRedReefPose(getLeftPose(pose)),
                getRedReefPose(getRightPose(pose)));
      }
    }
  }

  public static class AutoConstants {

    // Choreo
    public static final PIDController kXController_Choreo =
        new PIDController(8, 0, 0); // TODO: Tune
    public static final PIDController kYController_Choreo =
        new PIDController(8, 0, 0); // TODO: Tune
    public static final PIDController kThetaController_Choreo =
        new PIDController(6, 0, 0); // TODO: Tune

    // Repulsor
    public static final PIDController kXController_Repulsor =
        new PIDController(100, 0, 0); // TODO: Tune
    public static final PIDController kYController_Repulsor =
        new PIDController(100, 0, 0); // TODO: Tune
    public static final PIDController kThetaController_Repulsor =
        new PIDController(100, 0, 0); // TODO: Tune

    // Position PID
    public static final PIDController kXController_Position =
        new PIDController(3.8, 0, 0); // TODO: Tune
    public static final PIDController kYController_Position =
        new PIDController(3.8, 0, 0); // TODO: Tune
    public static final PIDController kThetaController_Position =
        new PIDController(8, 0, 0); // TODO: Tune
  }

  public static class VisionConstants {

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

    public static CameraInfo camera1Info =
        new CameraInfo(
            "White_Reef",
            new Transform3d(
                new Translation3d(-0.320048, -0.300306, 0.299816),
                new Rotation3d(0, Units.degreesToRadians(0), Math.PI - Units.degreesToRadians(55))),
            Rotation2d.fromDegrees(79.19),
            new int[] {1280, 800});

    public static CameraInfo camera2Info =
        new CameraInfo(
            "Black_Reef",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(3), Units.inchesToMeters(4), Units.inchesToMeters(9)),
                new Rotation3d(0, Units.degreesToRadians(0), Math.PI + Units.degreesToRadians(20))),
            Rotation2d.fromDegrees(94.9),
            new int[] {1600, 1200});

    public static CameraInfo camera3Info =
        new CameraInfo(
            "White_Station",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(7), Units.inchesToMeters(4.1), Units.inchesToMeters(9)),
                new Rotation3d(0, Units.degreesToRadians(0), 0)),
            Rotation2d.fromDegrees(79.76),
            new int[] {1280, 800});

    public static final Matrix<N3, N1> singleTagStdDev =
        VecBuilder.fill(0.4, 0.4, Double.MAX_VALUE);
    public static final Matrix<N3, N1> multiTagStdDev = VecBuilder.fill(0.2, 0.2, Double.MAX_VALUE);
  }

  public static class Motors {
    public static double FalconRPS =
        Units.radiansPerSecondToRotationsPerMinute(DCMotor.getFalcon500(1).freeSpeedRadPerSec) / 60;
  }

  public static class Swerve {

    public enum DriveGearing {
      L1(19d / 25d),
      L2(17d / 27d),
      L3(16d / 28d);

      public double reduction;

      DriveGearing(double reduction) {
        this.reduction = reduction * (50d / 14d) * (45d / 15d);
      }
    }

    public static double WheelDiameter = Units.inchesToMeters(4);
    public static double TrackWidth = Units.inchesToMeters(29 - 5.25);
    public static double TrackLength = Units.inchesToMeters(29 - 5.25);

    public static Translation2d[] ModulePositions =
        new Translation2d[] {
          new Translation2d(TrackWidth / 2, TrackLength / 2),
          new Translation2d(TrackWidth / 2, -TrackLength / 2),
          new Translation2d(-TrackWidth / 2, TrackLength / 2),
          new Translation2d(-TrackWidth / 2, -TrackLength / 2)
        };

    public static double TurnGearing = 150d / 7d;

    public static class ModuleInformation {

      public String name;
      public int driveID;
      public int turnID;
      public int encoderID;

      public ModuleInformation(String name, int driveID, int turnID, int encoderID) {
        this.name = name;
        this.driveID = driveID;
        this.turnID = turnID;
        this.encoderID = encoderID;
      }

      public static ModuleInformation frontLeft =
          new ModuleInformation(
              "Front Left ", CAN.Swerve_FL_D.id, CAN.Swerve_FL_T.id, CAN.Swerve_FL_E.id);
      public static ModuleInformation frontRight =
          new ModuleInformation(
              "Front Right ", CAN.Swerve_FR_D.id, CAN.Swerve_FR_T.id, CAN.Swerve_FR_E.id);
      public static ModuleInformation backLeft =
          new ModuleInformation(
              "Back Left ", CAN.Swerve_BL_D.id, CAN.Swerve_BL_T.id, CAN.Swerve_BL_E.id);
      public static ModuleInformation backRight =
          new ModuleInformation(
              "Back Right ", CAN.Swerve_BR_D.id, CAN.Swerve_BR_T.id, CAN.Swerve_BR_E.id);
    }
  }

  public static class Intake {

    public static double pivotGearing = (60d / 1) * (72d / 28);
    public static double maxAngle = Units.degreesToRadians(124);
    public static double minAngle = Units.degreesToRadians(10);

    public static double coralScoreAngle = Units.degreesToRadians(100);
    public static double algaeScoreAngle = Units.degreesToRadians(124);

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

    public static final double kSupplyLimit = 30;
    public static final double kStatorLimit = 60;

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

    public static double kAlgaeStrength = 0.9;
    public static int algaeTimerLength = 50; // 50 frames per second

    public static Slot0Configs motorSlot0 =
        new Slot0Configs()
            .withKS(0) // Volts
            .withKG(0.35) // Volts
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(4.5)
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

    public static double kMaxVelocity = 80; // Inches/s of Carriage Travel
    public static double kMaxAcceleration = 100; // Inches/s/s of Carriage Travel

    public static MotionMagicConfigs kMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity((kMaxVelocity / stages) / (sprocketPD * Math.PI))
            .withMotionMagicAcceleration((kMaxAcceleration / stages) / (sprocketPD * Math.PI));
  }

  public static final class Climber {
    public static double maxAngle = Units.degreesToRadians(90); // TODO: Verify
    public static double minAngle = Units.degreesToRadians(0); // This should be good, right?
    public static double gearing =
        (20d / 1) * (72d / 28); // TODO: Make sure the intake doesn't go inside the robot
    public static int currentLimit = 40;
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

    /**
     * TODO: this code is a carbon copy from intake so make sure this actually makes the motor work
     * as intended
     */
    public static Slot0Configs kPivotSlot0 =
        new Slot0Configs()
            .withKS(0)
            .withKV(12d / ((6380d / 60) * gearing))
            .withKP(70)
            .withKI(0)
            .withKD(0);

    // TODO: same here
    public static MotionMagicConfigs kPivotMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Units.degreesToRotations(400))
            .withMotionMagicAcceleration(Units.degreesToRotations(1200));
  }
}
