package frc.robot.subsystems.drive;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ColorSensorV3 m_colorSensor;

  public boolean hasNote() {
    Color noteColor = m_colorSensor.getColor();
    return noteColor == Color.kOrange;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note detected", hasNote());
    SmartDashboard.putNumber("Distance", m_colorSensor.getProximity());
    Logger.recordOutput("Note detected", hasNote());
    Logger.recordOutput("Distance", m_colorSensor.getProximity());
  }
}
