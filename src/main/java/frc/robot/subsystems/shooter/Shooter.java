package frc.robot.subsystems.shooter;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final Color noteColor = new Color(0,0,0);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  noteColor = m_colorSensor.getColor();

  String colorString;
  ColorMatchResult match = m_colorMatcher.matchClosestColor(noteColor);

  public boolean noteDetected(){
    return noteColor == Color.kOrange;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note Detected", noteDetected());
    SmartDashboard.putNumber("Distance", m_colorSensor.getProximity());
  }
}
