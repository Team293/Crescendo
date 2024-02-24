package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private static final int LAUNCHER_MOTOR_ID = 13;
  private static final double LAUNCHER_SET_SPEED = 41.0; // goes above this speed because of the PID
  private static final double LAUNCHER_READY_THRESHOLD = 2.0;

  private final LauncherIOTalonFX launchMotor;
  private final LauncherIOInputsAutoLogged launchMotorInputs = new LauncherIOInputsAutoLogged();
  private final ColorSensorIO colorSensorIO;
  private final ColorSensorIOInputsAutoLogged colorSensorInputs =
      new ColorSensorIOInputsAutoLogged();

  public Launcher() {
    colorSensorIO = new ColorSensorIORevV3();
    launchMotor = new LauncherIOTalonFX(LAUNCHER_MOTOR_ID);
  }

  public boolean noteDetected() {
    // Updated when m_sensorIO.updateInputs(m_sensorInputs) happens in periodic
    return (colorSensorInputs.IsNoteDetected);
  }

  @Override
  public void periodic() {
    // Update the color sensor and vision inputs
    launchMotor.updateInputs(launchMotorInputs);
    colorSensorIO.updateInputs(colorSensorInputs);

    Logger.processInputs("Launcher/Sensor", colorSensorInputs);
    Logger.processInputs("Launcher/Motor", launchMotorInputs);
    Logger.recordOutput("Launcher/Ready", isLauncherReady());
  }

  public void enableLauncher() {
    launchMotor.setVelocityRPS(LAUNCHER_SET_SPEED);
  }

  public void disableLauncher() {
    launchMotor.setVelocityRPS(0);
  }

  public double getVelocityRPS() {
    return launchMotorInputs.mechanismVelocityRotationsPerSec;
  }

  public boolean isLauncherReady() {
    return (launchMotorInputs.mechanismVelocityRotationsPerSec
        > (LAUNCHER_SET_SPEED - LAUNCHER_READY_THRESHOLD));
  }

  public boolean isLauncherNotReady() {
    return (launchMotorInputs.mechanismVelocityRotationsPerSec
        < (LAUNCHER_SET_SPEED - LAUNCHER_READY_THRESHOLD));
  }
}
