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

  /**
   * Change the setpoint of the climber
   *
   * @param setpoint Rotations
   * @return
   */
  public Command changeSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(
              MathUtil.clamp(
                  setpoint,
                  Constants.ClimberConstants.minRotations,
                  Constants.ClimberConstants.maxRotations));
        });
  }

  /**
   * Set the voltage of the climber
   *
   * @param volts Volts
   * @return
   */
  public Command setVoltage(double volts) {
    return this.runOnce(() -> io.setVoltage(volts));
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.setpoint, inputs.position, 1);
  }
}
