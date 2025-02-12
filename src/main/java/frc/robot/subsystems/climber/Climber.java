// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  public final ClimberIO io;
  public final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command changeSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(
              MathUtil.clamp(
                  setpoint, Constants.Climber.minRotations, Constants.Climber.maxRotations));
        });
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.setpoint, inputs.position, 1);
  }
}
