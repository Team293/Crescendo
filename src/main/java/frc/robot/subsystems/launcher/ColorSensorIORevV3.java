// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.launcher;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/** IO implementation for NavX */
// Note that ColorSensorIORevV3 inherits and adds onto ColorSensorIO
public class ColorSensorIORevV3 implements ColorSensorIO {
  private final I2C.Port m_i2cPort;
  private final ColorSensorV3 m_colorSensor;
  private final Color m_noteColor = Color.kOrange; // Is this correct?
  private final ColorMatch m_colorMatcher = new ColorMatch();

  public ColorSensorIORevV3() {
    m_i2cPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(m_i2cPort);
    m_colorMatcher.addColorMatch(m_noteColor);
    m_colorMatcher.setConfidenceThreshold(0.95);
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.IsConnected = true;
    inputs.Red = m_colorSensor.getRed();
    inputs.Green = m_colorSensor.getGreen();
    inputs.Blue = m_colorSensor.getBlue();
    inputs.Proximity = m_colorSensor.getProximity();

    // look at matchColor
    ColorMatchResult result = m_colorMatcher.matchColor(m_colorSensor.getColor());
    inputs.MatchResultColorRed = result.color.red;
    inputs.MatchResultColorBlue = result.color.blue;
    inputs.MatchResultColorGreen = result.color.green;
    inputs.MatchResultConfidence = result.confidence;
    inputs.MatchResultColorString = result.color.toString();
    inputs.IsNoteDetected = (result.color == m_noteColor);
  }
}
