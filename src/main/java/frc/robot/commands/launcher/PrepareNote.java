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

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class PrepareNote extends Command {
  private static final double INTAKE_SPEED_RPS = -5.0;
  private static final double TIME_TO_WAIT_SEC = 0.1;

  private final Intake intake;
  private final Launcher launcher;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization command. */
  public PrepareNote(Intake intake, Launcher launcher) {
    this.intake = intake;
    this.launcher = launcher;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (launcher.noteDetected()) {
    intake.setVelocity(INTAKE_SPEED_RPS);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !launcher.noteDetected() || timer.hasElapsed(TIME_TO_WAIT_SEC);
    return timer.hasElapsed(TIME_TO_WAIT_SEC);
  }
}
