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

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Intake extends SubsystemBase {
  private final IntakeIOTalonFX feedMotor;
  private final IntakeIOTalonFX launchMotor;

  private final Drive drive;

  private final IntakeIOInputsAutoLogged feedMotorInputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOInputsAutoLogged launchMotorInputs = new IntakeIOInputsAutoLogged();

  private double feedSetSpeed = 0.0;

  public Intake(Drive drive) {
    this.drive = drive;
    feedMotor = new IntakeIOTalonFX(10);
    launchMotor = new IntakeIOTalonFX(11);

    SmartDashboard.setDefaultNumber("Intake Speed Setpoint", 0.0);
  }

  public void periodic() {
    feedSetSpeed = SmartDashboard.getNumber("Intake Speed Setpoint", feedSetSpeed);

    double robotSpeed = drive.getCharacterizationVelocity();

    feedMotor.updateInputs(feedMotorInputs, robotSpeed);
    launchMotor.updateInputs(launchMotorInputs, robotSpeed);
  }

  public void enableIntake() {
    feedMotor.setSpeed(feedSetSpeed);
  }

  public void disableIntake() {
    feedMotor.setSpeed(0.0);
  }
}
