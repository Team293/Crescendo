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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

/** IO implementation for NavX */
// Note that ColorSensorIORevV3 inherits and adds onto ColorSensorIO
public class RiteSightSensor implements ColorSensorIO {
  private final DigitalInput m_sensor;
  private final Timer noteDetectionTimer = new Timer();

  public RiteSightSensor(int channel) {
    m_sensor = new DigitalInput(channel);
    noteDetectionTimer.start();
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.IsConnected = true;

    inputs.Red = 0.0d;
    inputs.Green = 0.0d;
    inputs.Blue = 0.0d;
    inputs.Proximity = 0;
    boolean signal = m_sensor.get();

    if (signal) {
      inputs.IsNoteDetected = true;
    } else {
      inputs.IsNoteDetected = false;
      noteDetectionTimer.reset();
      noteDetectionTimer.start();
    }

    inputs.detectedForSeconds = noteDetectionTimer.get();
  }
}
