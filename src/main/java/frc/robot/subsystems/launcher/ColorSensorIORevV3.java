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
  private final Color m_noteColor = new Color(0.520, 0.378, 0.103); // Is this correct?
  private final ColorMatch m_colorMatcher = new ColorMatch();

  public ColorSensorIORevV3() {
    m_i2cPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(m_i2cPort);

    m_colorMatcher.setConfidenceThreshold(0.95);
    m_colorMatcher.addColorMatch(m_noteColor);
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.IsConnected = true;
    Color detectedColor = m_colorSensor.getColor();
    inputs.Red = detectedColor.red;
    inputs.Green = detectedColor.green;
    inputs.Blue = detectedColor.blue;
    inputs.Proximity = m_colorSensor.getProximity();

    /** Run the color match algorithm on our detected color */
    ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);

    if (match == null) {
      inputs.IsNoteDetected = false;
      inputs.MatchResultConfidence = 0;
    } else {
      if (inputs.Proximity >= 700) {
        inputs.IsNoteDetected = true;
      } else {
        inputs.IsNoteDetected = false;
      }
      inputs.MatchResultConfidence = match.confidence;
    }
  }
}
