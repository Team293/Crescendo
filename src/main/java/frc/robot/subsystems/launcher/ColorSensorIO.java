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

import org.littletonrobotics.junction.AutoLog;

public interface ColorSensorIO {
  // @Autolog will automatically generate the specialized auto logging class in
  // build\generated\sources\annotationProcessor\java\main\frc\robot\subsystems\foo
  @AutoLog
  // The ColorSensorIOInputs class contains the variables that will be logged and shown within
  // AdvantageKit
  public static class ColorSensorIOInputs {
    // The variables that will be logged are declared here
    public boolean IsConnected;
    public int Red;
    public int Green;
    public int Blue;
    public int Proximity;
    public double MatchResultConfidence;
    public double MatchResultColorRed;
    public double MatchResultColorBlue;
    public double MatchResultColorGreen;
    public String MatchResultColorString;
    public boolean IsNoteDetected;
  }

  // updateInputs should be called in periodic.
  // This is where the members above are updated.
  // This should be @Override'd with the actual implementation in the inheriting class.
  // This should not be updated here
  public default void updateInputs(ColorSensorIOInputs inputs) {}
}
