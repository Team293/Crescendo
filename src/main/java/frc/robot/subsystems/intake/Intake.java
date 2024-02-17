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

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIOTalonFX feedMotor;
  private final Drive drive;
  private final IntakeIOInputsAutoLogged feedMotorInputs = new IntakeIOInputsAutoLogged();
  private double feedSetSpeed = 0.0;

  public Intake(Drive drive) {
    this.drive = drive;
    feedMotor = new IntakeIOTalonFX(10);

    SmartDashboard.setDefaultNumber("Intake Speed Setpoint(RPS)", 0.0);
  }

  public void periodic() {
    feedSetSpeed = SmartDashboard.getNumber("Intake Speed Setpoint(RPS)", feedSetSpeed);

    ChassisSpeeds robotSpeed = drive.getChassisSpeed();
    double speed = robotSpeed.vxMetersPerSecond;

    feedMotor.updateInputs(feedMotorInputs, speed);
    Logger.processInputs("Intake/Motor", feedMotorInputs);
  }

  public void enableIntake() {
    double appliedSpeed = -(feedSetSpeed + feedMotorInputs.robotSpeed);
    feedMotor.setSpeed(appliedSpeed);
  }

  public void disableIntake() {
    feedMotor.setSpeed(0.0);
  }
}
