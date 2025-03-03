package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.Swerve.ModuleConstants;

public class ModuleIO_Sim implements ModuleIO {

  private static final double loopPeriod = 0.02;
  private final ModuleConstants constants;

  private static final DCMotor driveMotor = DCMotor.getFalcon500(1);
  private static final DCMotor turnMotor = DCMotor.getFalcon500(1);

  private final TalonFX drive;
  private final TalonFX turn;

  public ModuleIO_Sim(ModuleConstants constants) {
    this.constants = constants;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {}
}
