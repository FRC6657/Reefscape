package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefSlot;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.de_algaefier.De_algaefier;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  // Subsystems
  Swerve drivebase;
  Elevator elevator;
  Outtake outtake;
  Intake intake;
  De_algaefier dealg;
  Climber climber;

  @AutoLogOutput(key = "RobotStates/Selected Reef")
  private String selectedReef = "Left"; // Selected Reef Pole

  @AutoLogOutput(key = "RobotStates/Elevator Level")
  private int elevatorLevel = 2; // Selected Reef Level

  @AutoLogOutput(key = "RobotStates/Selected Piece")
  private String selectedPiece = "Coral";

  // Array for easily grabbing setpoint heights.
  private double[] elevatorSetpoints = {
    0,
    0,
    Units.inchesToMeters(15.5), // L2
    Units.inchesToMeters(31.5), // L3
    Units.inchesToMeters(57.5), // L4
    // Here on out is for algae
    Units.inchesToMeters(5), // L2
    Units.inchesToMeters(20), // L3
    Constants.Elevator.maxHeight // L4 + Algae mode = score on barge
  };

  // Constructor
  public Superstructure(
      Swerve drivebase,
      Elevator elevator,
      Outtake outtake,
      Intake intake,
      De_algaefier dealg,
      Climber climber) {
    this.drivebase = drivebase;
    this.elevator = elevator;
    this.outtake = outtake;
    this.intake = intake;
    this.dealg = dealg;
    this.climber = climber;
  }

  /*
   * Grab and Log the 3D positions of all robot mechanisms for 3D visualization.
   */
  public void update3DPose() {
    Pose3d[] mechanismPoses = new Pose3d[5];
    mechanismPoses[0] = intake.get3DPose();
    mechanismPoses[1] = elevator.get3DPoses()[0];
    mechanismPoses[2] = elevator.get3DPoses()[1];
    mechanismPoses[3] = elevator.get3DPoses()[2];
    mechanismPoses[4] =
        new Pose3d(
            elevator
                .get3DPoses()[2]
                .getTranslation()
                .plus(new Translation3d(-0.330200, 0, 0.591850)),
            new Rotation3d(0, -Units.rotationsToRadians(dealg.getPosition()), 0));

    Logger.recordOutput("3D Poses", mechanismPoses);
  }

  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }

  // Gets the closest reef sector to the robot.
  @AutoLogOutput(key = "RobotStates/Nearest Reef")
  public Pose2d getNearestReef() {

    // Grab the alliance color
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Create an array of reef slots based on the alliance color
    ReefSlot[] reefSlots = new ReefSlot[6];

    // Set the reef slots based on the alliance color
    if (alliance == Alliance.Red) {
      reefSlots =
          new ReefSlot[] {
            Constants.FieldConstants.ReefPoses.Reef_1.red,
            Constants.FieldConstants.ReefPoses.Reef_2.red,
            Constants.FieldConstants.ReefPoses.Reef_3.red,
            Constants.FieldConstants.ReefPoses.Reef_4.red,
            Constants.FieldConstants.ReefPoses.Reef_5.red,
            Constants.FieldConstants.ReefPoses.Reef_6.red
          };
    } else {
      reefSlots =
          new ReefSlot[] {
            Constants.FieldConstants.ReefPoses.Reef_1.blue,
            Constants.FieldConstants.ReefPoses.Reef_2.blue,
            Constants.FieldConstants.ReefPoses.Reef_3.blue,
            Constants.FieldConstants.ReefPoses.Reef_4.blue,
            Constants.FieldConstants.ReefPoses.Reef_5.blue,
            Constants.FieldConstants.ReefPoses.Reef_6.blue
          };
    }

    // Create a list of reef middle poses the robot scores offcenter but this is
    // used instead just
    // to select the sector.
    List<Pose2d> reefMiddles = new ArrayList<>();
    for (ReefSlot reefSlot : reefSlots) {
      reefMiddles.add(reefSlot.middle);
    }

    Pose2d currentPos = drivebase.getPose(); // Get the current robot pose
    Pose2d nearestReefMiddle = currentPos.nearest(reefMiddles); // Get the nearest reef middle pose
    ReefSlot nearestReefSlot =
        reefSlots[
            reefMiddles.indexOf(
                nearestReefMiddle)]; // Get the reef slot of the nearest reef middle pose

    // Return the reef slot based on the selected reef
    if (selectedPiece == "Coral") {
      if (selectedReef == "Left") {
        return nearestReefSlot.left;
      } else if (selectedReef == "Right") {
        return nearestReefSlot.right;
      }
    } else {
      return nearestReefSlot.algae;
    }

    // If the selected reef is invalid return the robot's current pose.
    Logger.recordOutput("Errors", "Invalid Reef Selected '" + selectedReef + "'");
    return nearestReefMiddle;
  }

  // Simple command to change the selected reef level.
  public Command selectElevatorHeight(int height) {
    return Commands.runOnce(() -> elevatorLevel = height)
        .andThen(logMessage("Selected Elevator Height: " + height));
  }

  // Simple command to change the selected reef pole.
  public Command selectReef(String reef) {
    return Commands.runOnce(() -> this.selectedReef = reef)
        .andThen(logMessage("Selected Reef: " + reef));
  }

  // Select Coral Mode
  public Command selectPiece(String piece) {
    return Commands.runOnce(() -> selectedPiece = piece)
        .andThen(logMessage("Selected Piece: " + piece))
        .andThen(
            Commands.either(
                dealg.changeSetpoint(Constants.De_algaefier.minAngle),
                dealg.changeSetpoint(Constants.De_algaefier.maxAngle),
                () -> selectedPiece == "Coral"));
  }

  // Change Elevator Setpoint to the selected reef level.
  public Command raiseElevator() {
    return elevator
        .changeSetpoint(() -> elevatorSetpoints[elevatorLevel + (selectedPiece == "Coral" ? 0 : 3)])
        .andThen(
            logMessage(
                "Elevator Setpoint Changed To: "
                    + elevatorSetpoints[elevatorLevel]
                    + " Reef Level: "
                    + elevatorLevel));
  }

  public Command ElevatorIntake() {
    return Commands.either(
        Commands.sequence(
            logMessage("Elevator Intake"),
            outtake.changeRollerSetpoint(-0.5),
            Commands.waitUntil(outtake::coralDetected)
                .raceWith(
                    new NotifierCommand(
                        () -> {
                          if (outtake.coralDetected()) {
                            outtake.setRollerSetpoint(0);
                          }
                        },
                        250,
                        outtake)),
            outtake.changeRollerSetpoint(0)),
        Commands.sequence(
            logMessage("Elevator Algae Intake"), outtake.changeRollerSetpoint(-0.7) // TODO verify
            ),
        () -> selectedPiece == "Coral");
  }

  public Command PassiveElevatorIntake() {
    return Commands.either(
        outtake.changeRollerSetpoint(0),
        outtake.changeRollerSetpoint(-0.15),
        () -> selectedPiece == "Coral");
  }

  // Command for intaking game pieces from the ground
  public Command GroundIntake() {
    return Commands.either(
        Commands.sequence( // Coral
            logMessage("Ground Intake | Coral"),
            intake.changePivotSetpoint(Constants.Intake.maxAngle),
            intake.changeRollerSpeed(-Constants.Intake.kGroundIntakeSpeed)),
        Commands.sequence( // Algae
            logMessage("Ground Intake | Algae"),
            intake.changePivotSetpoint(Units.degreesToRadians(65)),
            intake.changeRollerSpeed(Constants.Intake.kGroundIntakeSpeed)),
        () -> selectedPiece == "Coral");
  }

  // Retracts the intake, while keeping a grip on the game piece
  public Command RetractIntake() {
    return Commands.either(
        Commands.sequence(
            logMessage("Retract Intake | Coral"),
            intake.changePivotSetpoint(Constants.Intake.minAngle),
            intake.changeRollerSpeed(-Constants.Intake.kFeedSpeed / 1.5)),
        Commands.sequence(
            logMessage("Retract Intake | Algae"),
            intake.changePivotSetpoint(Constants.Intake.minAngle),
            intake.changeRollerSpeed(Constants.Intake.kFeedSpeed)),
        () -> selectedPiece == "Coral");
  }

  // Scores a piece out of the ground intake.
  public Command GroundIntakeScore() {
    return Commands.either(
            Commands.sequence(
                logMessage("Ground Intake Score | Coral"),
                intake.changePivotSetpoint(Constants.Intake.coralScoreAngle),
                intake.changeRollerSpeed(Constants.Intake.kFeedSpeed)),
            Commands.sequence(
                logMessage("Ground Intake Score | Algae"),
                intake.changePivotSetpoint(Constants.Intake.algaeScoreAngle),
                intake.changeRollerSpeed(-Constants.Intake.kGroundIntakeSpeed)),
            () -> selectedPiece == "Coral")
        .andThen(
            Commands.sequence(
                Commands.waitSeconds(1.0),
                logMessage("Ground Intake Score | Retract"),
                intake.changePivotSetpoint(Constants.Intake.minAngle),
                intake.changeRollerSpeed(0)));
  }

  // Scores a coral (or algae) from the elevator
  public Command ElevatorScore() {
    return Commands.either(
        Commands.sequence(
            logMessage("Elevator Score"),
            outtake.changeRollerSetpoint(-0.3),
            Commands.waitUntil(() -> !outtake.coralDetected()).unless(RobotBase::isSimulation),
            Commands.waitSeconds(0.3),
            outtake.changeRollerSetpoint(0)),
        Commands.sequence(
            logMessage("Elevator Score"),
            outtake.changeRollerSetpoint(1.0),
            Commands.waitSeconds(0.5),
            outtake.changeRollerSetpoint(0)),
        () -> selectedPiece == "Coral");
  }

  // Scores a piece.
  // If the elevator is up it will score from the elevator, otherwise it will
  // score from the ground
  // intake.
  public Command Score() {
    return Commands.either(GroundIntakeScore(), ElevatorScore(), elevator::isDown);
  }

  // Raises the climer.
  public Command RaiseClimber() {
    return Commands.sequence(
        logMessage("Raising Climber"),
        elevator.changeSetpoint(Units.inchesToMeters(20)),
        dealg.changeSetpoint(Units.degreesToRotations(30)),
        intake.changePivotSetpoint(Units.degreesToRadians(50)),
        climber.setVoltage(-11));
  }

  // Lowers the climer.
  public Command LowerClimber() {
    return Commands.sequence(logMessage("Lowering Climber"), climber.setVoltage(6));
  }

  // Stows all mechanisms, and stops all rollers.
  public Command HomeRobot() {
    return Commands.sequence(
        logMessage("Home Robot"),
        outtake.changeRollerSetpoint(0),
        elevator.changeSetpoint(0),
        intake.changePivotSetpoint(Constants.Intake.minAngle),
        intake.changeRollerSpeed(0),
        Commands.either(
            dealg.changeSetpoint(Constants.De_algaefier.minAngle),
            dealg.changeSetpoint(Constants.De_algaefier.maxAngle),
            () -> selectedPiece == "Coral"));
  }

  /** Auto aim wrapper command. Used to select level and pole side before auto aiming. */
  public Command AutoAim(int coralLevel, String reefPole, boolean lead) {
    return Commands.sequence(
        selectPiece("Coral"),
        selectElevatorHeight(coralLevel),
        selectReef(reefPole),
        AutoAim(lead));
  }

  /**
   * General Purpose Auto Aim Command.
   *
   * <p>Will use the selected reef pole and level to aim at the nearest reef sector. In algae mode
   * it will select the correct elevator height based on the nearest reef sector.
   */
  public Command AutoAim(boolean lead) {
    return Commands.sequence(
        // Select Elevator Height If In Algae Mode
        Commands.sequence(
                Commands.either(
                    selectElevatorHeight(2),
                    selectElevatorHeight(3),
                    () -> {
                      Pose2d nearestReef = getNearestReef();
                      return nearestReef == Constants.FieldConstants.ReefPoses.Reef_1.blue.algae
                          || nearestReef == Constants.FieldConstants.ReefPoses.Reef_3.blue.algae
                          || nearestReef == Constants.FieldConstants.ReefPoses.Reef_5.blue.algae
                          || nearestReef == Constants.FieldConstants.ReefPoses.Reef_2.red.algae
                          || nearestReef == Constants.FieldConstants.ReefPoses.Reef_4.red.algae
                          || nearestReef == Constants.FieldConstants.ReefPoses.Reef_6.red.algae;
                    }),
                ElevatorIntake())
            .onlyIf(() -> selectedPiece == "Algae"),
        // Reset Auto Aim PID to reset the rate limiter
        drivebase.resetAutoAimPID(),
        // Drive towards the pose with a larger tolerance
        // While raising the elevator.
        Commands.parallel(
                drivebase
                    .goToPoseCoarse(
                        () -> getNearestReef().plus(new Transform2d(0.25, 0, new Rotation2d())),
                        new Constraints(3, 3),
                        new Constraints(Units.rotationsToRadians(2), Units.rotationsToRadians(4)))
                    .onlyIf(() -> lead),
                raiseElevator())
            // Approach the Reef Pole once the elevator is fully raised.
            // This has a tighter tolerance and slower speed.
            // If the robot is in algae mode it will just drive forward for a bit to grab the algae.
            // This is not PID controlled.
            .andThen(
                drivebase.goToPoseFine(
                    () ->
                        getNearestReef()
                            .plus(new Transform2d(Units.inchesToMeters(-1), 0, new Rotation2d())),
                    new Constraints(1, 1),
                    new Constraints(Units.rotationsToRadians(2), Units.rotationsToRadians(4)))));
  }

  public AutoRoutine DirectionTest(AutoFactory factory, boolean mirror) {

    String name = "DirectionTest";

    final AutoRoutine routine = factory.newRoutine(name);

    String mirrorFlag = mirror ? "mirrored_" : "";

    final AutoTrajectory one = routine.trajectory(mirrorFlag + name, 0);
    final AutoTrajectory two = routine.trajectory(mirrorFlag + name, 1);
    final AutoTrajectory three = routine.trajectory(mirrorFlag + name, 2);
    final AutoTrajectory four = routine.trajectory(mirrorFlag + name, 3);

    one.done()
        .onTrue(
            Commands.sequence(
                    outtake.changeRollerSetpoint(-0.5),
                    Commands.waitUntil(outtake::coralDetected),
                    outtake.changeRollerSetpoint(0),
                    new ScheduleCommand(two.cmd()))
                .asProxy());
    two.done().onTrue(three.cmd().asProxy());

    three.done().onTrue(Commands.sequence(Commands.waitSeconds(2), four.cmd()).asProxy());

    routine.active().onTrue(Commands.sequence(one.resetOdometry(), one.cmd()));

    return routine;
  }

  public Command AutonomousScoringSequence(int level, String reef) {
    return Commands.sequence(
        // Commands.runOnce(() -> drivebase.drive(new ChassisSpeeds()), drivebase),
        AutoAim(4, reef, true),
        Commands.waitUntil(elevator::atSetpoint),
        Score(),
        drivebase.driveVelocity(() -> new ChassisSpeeds(0.5, 0, 0)).withTimeout(0.125),
        Commands.runOnce(() -> drivebase.drive(new ChassisSpeeds()), drivebase),
        HomeRobot());
  }

  public AutoRoutine CenterL4(AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("CenterL4");

    final AutoTrajectory S_P1 = routine.trajectory("1 Piece Center", 0);
    final AutoTrajectory P1_Algae = routine.trajectory("1 Piece Center", 1);

    Command Start =
        Commands.sequence(
                AutonomousScoringSequence(4, "Right"),
                selectPiece("Algae"),
                drivebase.driveRR(() -> new ChassisSpeeds(0.4, 0, 0)).withTimeout(1.1),
                drivebase.driveRR(() -> new ChassisSpeeds(0, 0, 0)).withTimeout(0.01),
                selectElevatorHeight(2),
                AutoAim(true),
                ElevatorIntake(),
                new ScheduleCommand(P1_Algae.cmd()))
            .asProxy();

    P1_Algae.done().onTrue(Commands.sequence(ElevatorScore(), HomeRobot()).asProxy());

    routine.active().onTrue(Commands.sequence(S_P1.resetOdometry(), Start));

    return routine;
  }

  public AutoRoutine L4_3Piece(AutoFactory factory, boolean mirror) {

    final AutoRoutine routine = factory.newRoutine("3 Piece");

    String mirrorFlag = mirror ? "mirrored_" : "";

    final AutoTrajectory S_P1 = routine.trajectory(mirrorFlag + "3 Piece", 0);
    final AutoTrajectory P1_I1 = routine.trajectory(mirrorFlag + "3 Piece", 1);
    final AutoTrajectory I1_P2 = routine.trajectory(mirrorFlag + "3 Piece", 2);
    final AutoTrajectory P2_I2 = routine.trajectory(mirrorFlag + "3 Piece", 3);
    final AutoTrajectory I2_P3 = routine.trajectory(mirrorFlag + "3 Piece", 4);

    S_P1.atTimeBeforeEnd(0.775)
        .onTrue(
            Commands.sequence(
                    AutonomousScoringSequence(4, mirror ? "Right" : "Left"),
                    new ScheduleCommand(P1_I1.cmd()))
                .asProxy());

    P1_I1
        .done()
        .onTrue(
            Commands.sequence(
                    outtake.changeRollerSetpoint(-0.5),
                    Commands.waitUntil(outtake::coralDetected).withTimeout(3),
                    outtake.changeRollerSetpoint(0),
                    new ScheduleCommand(I1_P2.cmd()))
                .asProxy());

    I1_P2
        .atTimeBeforeEnd(0.75)
        .onTrue(
            Commands.sequence(
                    AutonomousScoringSequence(4, mirror ? "Right" : "Left"),
                    new ScheduleCommand(P2_I2.cmd()))
                .asProxy());

    P2_I2
        .done()
        .onTrue(
            Commands.sequence(
                    outtake.changeRollerSetpoint(-0.5),
                    Commands.waitUntil(outtake::coralDetected).withTimeout(3),
                    outtake.changeRollerSetpoint(0),
                    new ScheduleCommand(I2_P3.cmd()))
                .asProxy());

    I2_P3
        .atTimeBeforeEnd(0.75)
        .onTrue(AutonomousScoringSequence(4, mirror ? "Left" : "Right").asProxy());

    routine.active().onTrue(Commands.sequence(S_P1.resetOdometry(), S_P1.cmd()));

    return routine;
  }
}
