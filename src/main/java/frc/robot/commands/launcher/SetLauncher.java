package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;

public class SetLauncher extends Command {
  public Launcher launcher;
  public boolean active;

  public SetLauncher(Launcher launcher, boolean enabled) {
    this.launcher = launcher;
    this.active = enabled;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (active) {
      launcher.enableLauncher();
    } else {
      launcher.disableLauncher();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (active) {
      launcher.enableLauncher();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (active) {
      return launcher.isLauncherReady();
    } else {
      return true;
    }
  }
}
