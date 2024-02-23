package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final LauncherIOTalonFX launchMotor;
  private final LauncherIOInputsAutoLogged launchMotorInputs = new LauncherIOInputsAutoLogged();
  private final ColorSensorIO m_sensorIO;
  private final ColorSensorIOInputsAutoLogged m_sensorInputs = new ColorSensorIOInputsAutoLogged();

  private static double LAUNCHER_SET_SPEED = 37;

  public Launcher() {

    m_sensorIO = new ColorSensorIORevV3();
    // Initialize the ColorSensorIORevV3 object
    launchMotor = new LauncherIOTalonFX(13); // todo
  }

  public boolean noteDetected() {
    // Updated when m_sensorIO.updateInputs(m_sensorInputs) happens in periodic
    return (m_sensorInputs.IsNoteDetected);
  }

  @Override
  public void periodic() {
    // Update the color sensor and vision inputs
    launchMotor.updateInputs(launchMotorInputs);
    m_sensorIO.updateInputs(m_sensorInputs);
    LAUNCHER_SET_SPEED = SmartDashboard.getNumber("Launcher Speed Setpoint", LAUNCHER_SET_SPEED);

    Logger.processInputs("Launcher/Sensor", m_sensorInputs);
    Logger.processInputs("Launcher/Motor", launchMotorInputs);
  }

  public void enableLauncher() {
    launchMotor.setSpeed(LAUNCHER_SET_SPEED);
  }

  public void disableLauncher() {
    launchMotor.setSpeed(0);
  }
}
