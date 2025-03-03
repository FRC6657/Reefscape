// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class Robot extends LoggedRobot {

  private CommandXboxController driver = new CommandXboxController(0);

  public Robot() {
    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
