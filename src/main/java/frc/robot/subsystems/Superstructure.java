package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefSlot;
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

  @AutoLogOutput(key = "States/Selected Reef")
  private String selectedReef = "Left"; // Selected Reef Pole

  @AutoLogOutput(key = "States/Elevator Level")
  private int elevatorLevel = 2; // Selected Reef Level

  @AutoLogOutput(key = "States/Selected Piece")
  private String selectedPiece = "Coral";

  private double[] elevatorSetpoints = {
    0,
    0,
    Units.inchesToMeters(14),
    Units.inchesToMeters(30),
    Units.inchesToMeters(55) // Array for easily grabbing setpoint heights.
  };

  // Constructor
  public Superstructure(Swerve drivebase, Elevator elevator, Outtake outtake, Intake intake) {
    this.drivebase = drivebase;
    this.elevator = elevator;
    this.outtake = outtake;
    this.intake = intake;
  }

  /*
   * Grab and Log the 3D positions of all robot mechanisms for 3D visualization.
   */
  public void update3DPose() {
    Pose3d[] mechanismPoses = new Pose3d[4];
    mechanismPoses[0] = intake.get3DPose();
    mechanismPoses[1] = elevator.get3DPoses()[0];
    mechanismPoses[2] = elevator.get3DPoses()[1];
    mechanismPoses[3] = elevator.get3DPoses()[2];

    Logger.recordOutput("3D Poses", mechanismPoses);
  }

  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }

  // Gets the closest reef sector to the robot.
  @AutoLogOutput(key = "AutoAim/TargetPose")
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

    // Create a list of reef middle poses the robot scores offcenter but this is used instead just
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
    if (selectedReef == "Left") {
      return nearestReefSlot.left;
    } else if (selectedReef == "Right") {
      return nearestReefSlot.right;
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
        .andThen(logMessage("Selected Piece: " + piece));
  }

  // Change Elevator Setpoint to the selected reef level.
  public Command raiseElevator() {
    return elevator
        .changeSetpoint(() -> elevatorSetpoints[elevatorLevel])
        .andThen(
            logMessage(
                "Elevator Setpoint Changed To: "
                    + elevatorSetpoints[elevatorLevel]
                    + " Reef Level: "
                    + elevatorLevel));
  }

  // Command to algin to the reef and get ready to score a coral.
  // This command aligns the drivebase to the nearest reef and raises the elevator to the selected
  // reef level.
  // Command will end when the drivebase is aligned and the elevator is at the selected reef level.
  public Command ReefAlign(String side, int level) {
    return Commands.sequence(
        logMessage("Autonomous Reef Align | Side: " + side + " | Level:" + level),
        selectReef(side),
        selectElevatorHeight(level),
        Commands.parallel(
            drivebase.goToPose(() -> getNearestReef()),
            elevator
                .changeSetpoint((() -> elevatorSetpoints[elevatorLevel]))
                .andThen(Commands.waitUntil(elevator::atSetpoint))));
  }

  // Command for intaking coral from the human player station
  public Command ElevatorIntake() {
    return Commands.sequence(
        logMessage("Elevator Intake"),
        outtake.changeRollerSetpoint(-0.5),
        Commands.waitUntil(outtake::coralDetected),
        outtake.changeRollerSetpoint(0));
  }

  // Command for intaking game pieces from the ground
  public Command GroundIntake() {
    return Commands.either(
        Commands.sequence( // Coral
            logMessage("Ground Intake | Coral"),
            intake.changePivotSetpoint(Units.degreesToRadians(2)),
            intake.changeRollerSpeed(-Constants.Intake.kGroundIntakeSpeed)),
        Commands.sequence( // Algae
            logMessage("Ground Intake | Algae"),
            intake.changePivotSetpoint(Units.degreesToRadians(60)),
            intake.changeRollerSpeed(Constants.Intake.kGroundIntakeSpeed)),
        () -> selectedPiece == "Coral");
  }

  // Retracts the intake, while keeping a grip on the game piece
  public Command RetractIntake() {
    return Commands.either(
        Commands.sequence(
            logMessage("Retract Intake | Coral"),
            intake.changePivotSetpoint(Constants.Intake.maxAngle),
            intake.changeRollerSpeed(-Constants.Intake.kFeedSpeed / 1.5)),
        Commands.sequence(
            logMessage("Retract Intake | Algae"),
            intake.changePivotSetpoint(Constants.Intake.maxAngle),
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
                logMessage("Ground Intake Score | Retract"),
                intake.changePivotSetpoint(Constants.Intake.maxAngle),
                intake.changeRollerSpeed(0)));
  }

  // Scores a coral from the elevator
  public Command ElevatorScore() {
    return Commands.sequence(
        logMessage("Elevator Score"),
        outtake.changeRollerSetpoint(-0.4),
        Commands.waitUntil(() -> !outtake.coralDetected()).unless(RobotBase::isSimulation),
        outtake.changeRollerSetpoint(0));
  }

  // Scores a piece.
  // If the elevator is up it will score from the elevator, otherwise it will score from the ground
  // intake.
  public Command Score() {
    return Commands.either(GroundIntakeScore(), ElevatorScore(), elevator::isDown);
  }

  // Stows all mechanisms, and stops all rollers.
  public Command HomeRobot() {
    return Commands.sequence(
        logMessage("Home Robot"),
        outtake.changeRollerSetpoint(0),
        elevator.changeSetpoint(0),
        intake.changePivotSetpoint(Constants.Intake.maxAngle),
        intake.changeRollerSpeed(0));
  }

  public AutoRoutine L4_3Piece(AutoFactory factory, boolean mirror) {

    final AutoRoutine routine = factory.newRoutine("3 Piece");

    String mirrorFlag = mirror ? "mirrored_" : "";

    final AutoTrajectory S_P1 = routine.trajectory(mirrorFlag + "3 Piece", 0);
    final AutoTrajectory P1_I1 = routine.trajectory(mirrorFlag + "3 Piece", 1);
    final AutoTrajectory I1_P2 = routine.trajectory(mirrorFlag + "3 Piece", 2);
    final AutoTrajectory P2_I2 = routine.trajectory(mirrorFlag + "3 Piece", 3);
    final AutoTrajectory I2_P3 = routine.trajectory(mirrorFlag + "3 Piece", 4);

    S_P1.atTime("Score")
        .onTrue(
            Commands.sequence(
                ReefAlign(mirror ? "Right" : "Left", 4).asProxy(),
                Score().asProxy(),
                HomeRobot().asProxy(),
                Commands.waitUntil(elevator::nearSetpoint),
                new ScheduleCommand(P1_I1.cmd())));

    P1_I1
        .done()
        .onTrue(
            Commands.sequence(
                outtake.changeRollerSetpoint(-0.5).asProxy(),
                Commands.waitUntil(outtake::coralDetected).withTimeout(3).asProxy(),
                new ScheduleCommand(I1_P2.cmd())));

    I1_P2
        .atTime("Score")
        .onTrue(
            Commands.sequence(
                ReefAlign(mirror ? "Right" : "Left", 4).asProxy(),
                Score().asProxy(),
                HomeRobot().asProxy(),
                Commands.waitUntil(elevator::nearSetpoint),
                new ScheduleCommand(P2_I2.cmd())));

    P2_I2
        .done()
        .onTrue(
            Commands.sequence(
                outtake.changeRollerSetpoint(-0.5).asProxy(),
                Commands.waitUntil(outtake::coralDetected).withTimeout(3).asProxy(),
                new ScheduleCommand(I2_P3.cmd())));

    I2_P3
        .atTime("Score")
        .onTrue(
            Commands.sequence(
                ReefAlign(mirror ? "Left" : "Right", 4).asProxy(), Score().asProxy()));

    routine.active().onTrue(Commands.sequence(S_P1.resetOdometry(), S_P1.cmd()));

    return routine;
  }

  public AutoRoutine taxi(AutoFactory factory, boolean mirror) {
    final AutoRoutine routine = factory.newRoutine("Taxi");

    String mirrorFlag = mirror ? "mirrored_" : "";

    final AutoTrajectory S_Pos = routine.trajectory(mirrorFlag + "Taxi", 0);

    routine.active().onTrue(Commands.sequence(S_Pos.resetOdometry(), S_Pos.cmd()));

    return routine;
  }

  public AutoRoutine taxiMiddleL1(AutoFactory factory) {
    final AutoRoutine routine =
        factory.newRoutine(
            "Taxi Middle L1"); // no need for mirror since it is allined perfectly in the middle

    final AutoTrajectory S_P1 = routine.trajectory("Taxi Middle L1", 0);

    S_P1.done().onTrue(GroundIntakeScore().asProxy());

    routine.active().onTrue(Commands.sequence(S_P1.resetOdometry(), S_P1.cmd()));

    return routine;
  }

  public AutoRoutine onePiece(AutoFactory factory, boolean mirror) {

    final AutoRoutine routine = factory.newRoutine("One Piece");

    String mirrorFlag = mirror ? "mirrored_" : "";

    final AutoTrajectory S_P1 = routine.trajectory(mirrorFlag + "One Piece", 0);
    final AutoTrajectory P1_Pos = routine.trajectory(mirrorFlag + "One Piece", 1);

    S_P1.atTime("Score")
        .onTrue(
            Commands.sequence(
                ReefAlign(mirror ? "Right" : "Left", 4).asProxy(),
                Score().asProxy(),
                HomeRobot().asProxy(),
                Commands.waitUntil(elevator::nearSetpoint),
                new ScheduleCommand(P1_Pos.cmd()))); // ends auto out of the way

    routine.active().onTrue(Commands.sequence(S_P1.resetOdometry(), S_P1.cmd()));

    return routine;
  }

  public AutoRoutine TwoCoral180(AutoFactory factory, boolean mirror) {

    final AutoRoutine routine = factory.newRoutine("Two Coral 180 degree turn");

    String mirrorFlag = mirror ? "mirrored_" : "";

    final AutoTrajectory S_P1 = routine.trajectory(mirrorFlag + "Two Coral 180 degree turn", 0);
    final AutoTrajectory P1_I1 = routine.trajectory(mirrorFlag + "Two Coral 180 degree turn", 1);
    final AutoTrajectory I1_P2 = routine.trajectory(mirrorFlag + "Two Coral 180 degree turn", 2);
    final AutoTrajectory P2_Pos = routine.trajectory(mirrorFlag + "Two Coral 180 degree turn", 3);

    S_P1.atTime("Score")
        .onTrue(
            Commands.sequence(
                ReefAlign(mirror ? "Right" : "Left", 4).asProxy(),
                Score().asProxy(),
                HomeRobot().asProxy(),
                // Commands.waitUntil(elevator::nearSetpoint),
                new ScheduleCommand(P1_I1.cmd())));

    P1_I1
        .done()
        .onTrue(
            Commands.sequence(
                outtake.changeRollerSetpoint(-0.5).asProxy(),
                Commands.waitUntil(outtake::coralDetected).withTimeout(3).asProxy(),
                new ScheduleCommand(I1_P2.cmd())));

    I1_P2
        .atTime("Score")
        .onTrue(
            Commands.sequence(
                ReefAlign(mirror ? "Right" : "Left", 4).asProxy(),
                Score().asProxy(),
                HomeRobot().asProxy(),
                Commands.waitUntil(elevator::nearSetpoint),
                new ScheduleCommand(P2_Pos.cmd())));

    routine.active().onTrue(Commands.sequence(S_P1.resetOdometry(), S_P1.cmd()));

    return routine;
  }

  public AutoRoutine TwoCoralAdjacent(AutoFactory factory, boolean mirror) {

    final AutoRoutine routine = factory.newRoutine("Two Coral Adjacent");

    String mirrorFlag = mirror ? "mirrored_" : "";

    final AutoTrajectory S_P1 = routine.trajectory(mirrorFlag + "Two Coral Adjacent", 0);
    final AutoTrajectory P1_I1 = routine.trajectory(mirrorFlag + "Two Coral Adjacent", 1);
    final AutoTrajectory I1_P2 = routine.trajectory(mirrorFlag + "Two Coral Adjacent", 2);

    S_P1.atTime("Score")
        .onTrue(
            Commands.sequence(
                ReefAlign(mirror ? "Right" : "Left", 4).asProxy(),
                Score().asProxy(),
                HomeRobot().asProxy(),
                Commands.waitUntil(elevator::nearSetpoint),
                new ScheduleCommand(P1_I1.cmd())));

    P1_I1
        .done()
        .onTrue(
            Commands.sequence(
                outtake.changeRollerSetpoint(-0.5).asProxy(),
                Commands.waitUntil(outtake::coralDetected).withTimeout(3).asProxy(),
                new ScheduleCommand(I1_P2.cmd())));

    I1_P2
        .atTime("Score")
        .onTrue(
            Commands.sequence(
                ReefAlign(mirror ? "Right" : "Left", 4).asProxy(), Score().asProxy()));

    routine.active().onTrue(Commands.sequence(S_P1.resetOdometry(), S_P1.cmd()));

    return routine;
  }
}
