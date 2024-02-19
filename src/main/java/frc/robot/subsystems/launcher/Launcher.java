package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.ColorSensorIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final LauncherIOTalonFX launchMotor;
  private final LauncherIOInputsAutoLogged launchMotorInputs = new LauncherIOInputsAutoLogged();
  private final ColorSensorIO m_sensorIO;
  private final ColorSensorIOInputsAutoLogged m_sensorInputs = new ColorSensorIOInputsAutoLogged();



  private double launcherSetSpeed = 0.0d;

  public Launcher() {
    m_sensorIO = new ColorSensorIORevV3();
    // Initialize the ColorSensorIORevV3 object
    launchMotor = new LauncherIOTalonFX(13); // todo
    launcherSetSpeed = SmartDashboard.getNumber("launcher speed(RPS)", 0.0d); // rps
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
    launcherSetSpeed = SmartDashboard.getNumber("Launcher Speed Setpoint", launcherSetSpeed);

    Logger.processInputs("Launcher/Sensor", m_sensorInputs);
    Logger.processInputs("Launcher/Motor", launchMotorInputs);
  }

  public void enableLauncher() {
    launchMotor.setSpeed(launcherSetSpeed);
  }

  public void disableLauncher() {
    launchMotor.setSpeed(0);
  }
}
