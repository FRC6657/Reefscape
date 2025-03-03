package frc.robot.subsystems.drivebase;

import org.littletonrobotics.junction.Logger;

public class Module {

  // Module IOs
  private ModuleIO io;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO io) {
    this.io = io;
  }

  // Update Module IO
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
  }
}
