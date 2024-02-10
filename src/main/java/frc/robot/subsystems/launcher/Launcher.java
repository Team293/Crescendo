package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final ColorSensorIO m_sensorIO;
  private final ColorSensorIOInputsAutoLogged m_sensorInputs = new ColorSensorIOInputsAutoLogged();
  private final VisionIO m_visionIO;
  private final VisionIOInputsAutoLogged m_visionInputs = new VisionIOInputsAutoLogged();

  public Launcher() {
    // Initialize the ColorSensorIORevV3 object
    m_sensorIO = new ColorSensorIORevV3();
    m_visionIO = new VisionIOLimelight("limelight");
  }

  public boolean noteDetected() {
    // Updated when m_sensorIO.updateInputs(m_sensorInputs) happens in periodic
    return (m_sensorInputs.IsNoteDetected);
  }

  @Override
  public void periodic() {
    // Update the color sensor and vision inputs
    m_sensorIO.updateInputs(m_sensorInputs);
    m_visionIO.updateInputs(m_visionInputs);
    Logger.processInputs("Vision", m_visionInputs);
    Logger.processInputs("Launcher/Sensor", m_sensorInputs);
  }
}
