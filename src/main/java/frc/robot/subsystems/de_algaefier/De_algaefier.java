package frc.robot.subsystems.de_algaefier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class De_algaefier extends SubsystemBase {

  private final De_algaefierIO io;
  private final De_algaefierIOInputsAutoLogged inputs = new De_algaefierIOInputsAutoLogged();


  /** Creates a new De_algaefier. */
  public De_algaefier(De_algaefierIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
