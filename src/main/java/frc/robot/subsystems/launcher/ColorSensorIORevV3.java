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

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;

/** IO implementation for NavX */
// Note that ColorSensorIORevV3 inherits and adds onto ColorSensorIO
public class ColorSensorIORevV3 implements ColorSensorIO {
  private final I2C.Port m_i2cPort;
  private final ColorSensorV3 m_colorSensor;
  private final Color noteColor = Color.kOrange; //Is this correct?
  private final ColorMatch m_colorMatcher = new ColorMatch();

  public ColorSensorIORevV3() {
    m_i2cPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(m_i2cPort);
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.IsConnected = true;
    inputs.Red = ???;
    inputs.Green = ???;
    inputs.Blue = ???;
    inputs.Proximity = ???;

    //look at matchColor
    inputs.ColorMatchResult = ???; 
    inputs.IsNoteDetected = ???;
  }
}
