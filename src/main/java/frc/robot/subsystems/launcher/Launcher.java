package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final ColorSensorIO m_sensorIO;
  private final ColorSensorIOInputsAutoLogged m_sensorInputs = new ColorSensorIOInputsAutoLogged();
  private final LauncherIOTalonFX launchMotor;
  private final LauncherIOInputsAutoLogged launchMotorInputs = new LauncherIOInputsAutoLogged();

  private double feedSetSpeed = 0.0;

  public Launcher() {
    // Initialize the ColorSensorIORevV3 object
    m_sensorIO = new ColorSensorIORevV3();

    launchMotor = new LauncherIOTalonFX(13); // todo
  }

  public boolean noteDetected() {
    // Updated when m_sensorIO.updateInputs(m_sensorInputs) happens in periodic
    return (m_sensorInputs.IsNoteDetected);
  }

  @Override
  public void periodic() {
    // Update the color sensor and vision inputs
    m_sensorIO.updateInputs(m_sensorInputs);
    launchMotor.updateInputs(launchMotorInputs);

    feedSetSpeed = SmartDashboard.getNumber("Launcher Speed Setpoint", feedSetSpeed);

    Logger.processInputs("Launcher/Sensor", m_sensorInputs);
    Logger.processInputs("Launcher/Motor", launchMotorInputs);
    Logger.recordOutput("Launcher/FeedSetPoint", feedSetSpeed);
  }

  public void enableLauncher() {
    launchMotor.setSpeed(feedSetSpeed);
  }

  public void disableLauncher() {
    launchMotor.setSpeed(0);
  }
}
