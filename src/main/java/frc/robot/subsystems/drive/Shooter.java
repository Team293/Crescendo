package frc.robot.subsystems.drive;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private ColorSensorV3 m_colorSensor;

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("Note detected", hasNote());
    SmartDashboard.putNumber("Distance", m_colorSensor.getProximity());
  }
  
  public boolean hasNote() {
    Color noteColor = m_colorSensor.getColor();
    return noteColor == Color.kOrange;
  }
}
