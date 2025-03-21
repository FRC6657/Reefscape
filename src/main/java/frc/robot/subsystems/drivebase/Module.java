package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {

  // Module IOs
  private ModuleIO io;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io) {
    this.io = io;
  }

  /**
   * Runs the module with the given state
   *
   * @param state The state to run the module with
   * @return The optimized module state being ran
   */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    state.optimize(inputs.turnPosition);
    io.setTurnSetpoint(state.angle);
    io.setDriveSetpoint(state.speedMetersPerSecond);
    return state;
  }

  /**
   * @return the current state of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition);
  }

  /**
   * @return the current position of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
  }

  // Update Module IO
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs(
        new StringBuilder("Swerve/").append(inputs.prefix).append(" Module").toString(), inputs);

    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      odometryPositions[i] =
          new SwerveModulePosition(
              inputs.odometryDrivePositions[i], inputs.odometryTurnPositions[i]);
    }
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Reset the drive encoder to 0 */
  public void resetDriveEncoder() {
    io.resetDriveEncoder();
  }
}
