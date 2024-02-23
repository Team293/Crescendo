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

package frc.robot.commands.note;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import java.util.function.BooleanSupplier;

// A wrapper that abstracts all the commands needed to launch a note
public class LaunchNote extends Command {
  private Command launchNoteCommand;

  private Timer maxLength = new Timer();

  public LaunchNote(Intake intake, Launcher launcher) {
    launchNoteCommand =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new PrepareNote(intake, launcher), new SetLauncher(launcher, true)),
            new FeedNoteToLauncher(intake, launcher),
            new SetLauncher(launcher, false));
  }

  public LaunchNote(Intake intake, Launcher launcher, BooleanSupplier cancelCommand) {
    launchNoteCommand =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new PrepareNote(intake, launcher), new SetLauncher(launcher, true)),
            new FeedNoteToLauncher(intake, launcher),
            new SetLauncher(launcher, false));
    // launchNoteCommand.onlyWhile(() -> !cancelCommand.getAsBoolean());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchNoteCommand.schedule();
    maxLength.reset();
    maxLength.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // launchNoteCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the launcher
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (maxLength.hasElapsed(6.0)) {
      launchNoteCommand.cancel();
      return true;
    }
    ;
    return launchNoteCommand.isFinished();
  }
}
