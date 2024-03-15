package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  private static final int LAUNCHER_MOTOR_ID = 13;
  private static final double LAUNCHER_SET_SPEED = 41.0;
  private static final double LAUNCHER_READY_THRESHOLD = 2.0;

  private final LauncherIOTalonFX launchMotor;
  private final LauncherIOInputsAutoLogged launchMotorInputs = new LauncherIOInputsAutoLogged();
  private final ColorSensorIOInputsAutoLogged rightSightSensorInputs =
      new ColorSensorIOInputsAutoLogged();
  private final RiteSightSensor proximitySensorIO;

  public Launcher() {
    proximitySensorIO = new RiteSightSensor(0);
    launchMotor = new LauncherIOTalonFX(LAUNCHER_MOTOR_ID);
  }

  public boolean isNoteDetected() {
    // Updated when m_sensorIO.updateInputs(m_sensorInputs) happens in periodic
    return (rightSightSensorInputs.IsNoteDetected);
  }

  @Override
  public void periodic() {
    // Update the color sensor and vision inputs
    launchMotor.updateInputs(launchMotorInputs);
    proximitySensorIO.updateInputs(rightSightSensorInputs);

    // Logger.processInputs("Launcher/Sensor", colorSensorInputs);
    // Logger.processInputs("Launcher/Motor", launchMotorInputs);
    // Logger.recordOutput("Launcher/Ready", isReadyToShoot());
  }

  public void enableLauncher() {
    setVelocity(LAUNCHER_SET_SPEED);
  }

  public void reverseLauncher() {
    setVelocity(-5.0);
  }

  public void setVelocity(double velocityRPS) {
    launchMotor.setVelocityRPS(velocityRPS);
  }

  public void disableLauncher() {
    launchMotor.setVelocityRPS(0);
  }

  public double getVelocityRPS() {
    return launchMotorInputs.mechanismVelocityRotationsPerSec;
  }

  public boolean isReadyToShoot() {
    return (getVelocityRPS() > (LAUNCHER_SET_SPEED - LAUNCHER_READY_THRESHOLD));
  }

  public double detectedNoteForSeconds() {
    return rightSightSensorInputs.detectedForSeconds;
  }
}
