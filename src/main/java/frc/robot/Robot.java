// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO_Sim;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO_Real;
import frc.robot.subsystems.elevator.ElevatorIO_Sim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO_Real;
import frc.robot.subsystems.intake.IntakeIO_Sim;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO_Real;
import frc.robot.subsystems.outtake.OuttakeIO_Sim;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandGenericHID operator = new CommandGenericHID(1);

  private final Swerve swerve;
  private final Elevator elevator;
  private final Intake intake;
  private final Outtake outtake;

  private final Superstructure superstructure;
  private final AutoFactory autoFactory;

  private LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  public Robot() {

    swerve =
        new Swerve(
            RobotBase.isReal() ? new GyroIO_Real() : new GyroIO() {},
            new ModuleIO[] {
              RobotBase.isReal()
                  ? new ModuleIO_Real(Constants.Swerve.frontLeftModule)
                  : new ModuleIO_Sim(Constants.Swerve.frontLeftModule),
              RobotBase.isReal()
                  ? new ModuleIO_Real(Constants.Swerve.frontRightModule)
                  : new ModuleIO_Sim(Constants.Swerve.frontRightModule),
              RobotBase.isReal()
                  ? new ModuleIO_Real(Constants.Swerve.backLeftModule)
                  : new ModuleIO_Sim(Constants.Swerve.backLeftModule),
              RobotBase.isReal()
                  ? new ModuleIO_Real(Constants.Swerve.backRightModule)
                  : new ModuleIO_Sim(Constants.Swerve.backRightModule)
            });

    elevator = new Elevator(RobotBase.isReal() ? new ElevatorIO_Real() : new ElevatorIO_Sim());
    intake = new Intake(RobotBase.isReal() ? new IntakeIO_Real() : new IntakeIO_Sim());
    outtake = new Outtake(RobotBase.isReal() ? new OuttakeIO_Real() : new OuttakeIO_Sim());

    superstructure = new Superstructure(swerve, elevator, outtake, intake);

    autoFactory =
        new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);

    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Test", superstructure.DirectionTest(autoFactory, false).cmd());
    autoChooser.addOption("3 Piece", superstructure.L4_3Piece(autoFactory, false).cmd());
  }

  @Override
  public void robotInit() {
    Logger.recordMetadata("Arborbotics 2025", "Arborbotics 2025");
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }

    swerve.setDefaultCommand(
        swerve.driveTeleop(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.1)
                        * Constants.Swerve.maxLinearSpeed,
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.1)
                        * Constants.Swerve.maxLinearSpeed,
                    -MathUtil.applyDeadband(driver.getRightX(), 0.1)
                        * Constants.Swerve.maxAngularSpeed
                        * 0.5)));

    driver.a().whileTrue(superstructure.AutoAim());
    driver.a().onFalse(Commands.runOnce(() -> swerve.drive(new ChassisSpeeds())));

    operator.button(9).onTrue(superstructure.selectElevatorHeight(2));
    operator.button(8).onTrue(superstructure.selectElevatorHeight(3));
    operator.button(7).onTrue(superstructure.selectElevatorHeight(4));

    operator.button(2).onTrue(superstructure.selectPiece("Coral"));
    operator.button(5).onTrue(superstructure.selectPiece("Algae"));

    operator.button(6).onTrue(superstructure.selectReef("Left"));
    operator.button(3).onTrue(superstructure.selectReef("Right"));

    operator.button(4).onTrue(superstructure.HomeRobot());
    operator
        .button(1)
        .onTrue(outtake.changeRollerSetpoint(-0.8))
        .onFalse(outtake.changeRollerSetpoint(0));

    operator
        .button(1)
        .onTrue(outtake.changeRollerSetpoint(-0.8))
        .onFalse(outtake.changeRollerSetpoint(0));

    operator.button(4).onTrue(superstructure.HomeRobot());

    // Manual Elevator Controls
    driver.povUp().onTrue(superstructure.raiseElevator());
    driver.povDown().onTrue(elevator.changeSetpoint(0));

    // Ground Intake
    driver
        .rightTrigger()
        .onTrue(superstructure.GroundIntake())
        .onFalse(superstructure.RetractIntake());

    // HP Intake
    driver
        .rightBumper()
        .onTrue(superstructure.ElevatorIntake())
        .onFalse(outtake.changeRollerSetpoint(0));

    // General Score
    driver.leftTrigger().onTrue(superstructure.Score()).onFalse(superstructure.HomeRobot());

    Logger.start();
  }

  @Override
  public void robotPeriodic() {

    superstructure.update3DPose();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autoChooser.get().schedule();
  }

  @Override
  public void teleopInit() {
    autoChooser.get().cancel();
  }

  private Command rumble(double duration, double intensity) {
    return Commands.sequence(
        Commands.runOnce(() -> driver.setRumble(RumbleType.kRightRumble, intensity)),
        Commands.waitSeconds(duration),
        Commands.runOnce(() -> driver.setRumble(RumbleType.kRightRumble, 0)));
  }
}
