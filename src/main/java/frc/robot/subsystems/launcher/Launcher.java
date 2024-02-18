package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final LauncherIOTalonFX launchMotor;
  private final LauncherIOInputsAutoLogged launchMotorInputs = new LauncherIOInputsAutoLogged();

  private double launcherSetSpeed = 0.0d;

  public Launcher() {
    // Initialize the ColorSensorIORevV3 object
    launchMotor = new LauncherIOTalonFX(13); // todo
    launcherSetSpeed = SmartDashboard.getNumber("launcher speed(RPS)", 0.0d); // rps
  }

  @Override
  public void periodic() {
    // Update the color sensor and vision inputs
    launchMotor.updateInputs(launchMotorInputs);

    launcherSetSpeed = SmartDashboard.getNumber("Launcher Speed Setpoint", launcherSetSpeed);

    Logger.processInputs("Launcher/Motor", launchMotorInputs);
  }

  public void enableLauncher() {
    launchMotor.setSpeed(launcherSetSpeed);
  }

  public void disableLauncher() {
    launchMotor.setSpeed(0);
  }
}
