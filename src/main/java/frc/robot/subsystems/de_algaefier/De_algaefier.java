package frc.robot.subsystems.de_algaefier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class De_algaefier extends SubsystemBase {

  private final De_algaefierIO io;
  private final De_algaefierIOInputsAutoLogged inputs = new De_algaefierIOInputsAutoLogged();

  /** Creates a new De_algaefier. */
  public De_algaefier(De_algaefierIO io) {
    this.io = io;
  }

  public Command changeSetpoint(double rotations) {
    return this.runOnce(() -> io.changeSetpoint(rotations));
  }

  public double getPosition() {
    return inputs.kPosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Dealg", inputs);
  }
}
