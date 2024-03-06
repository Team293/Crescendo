package frc.robot.commands.note;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;

public class SetLauncher extends Command {
  // state variables
  private final boolean enabled;

  // subsystems
  private final Launcher launcher;

  public SetLauncher(Launcher launcher, boolean enabled) {
    this.launcher = launcher;
    this.enabled = enabled;

    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (enabled) {
      // Enable the launcher
      launcher.enableLauncher();
    } else {
      launcher.disableLauncher();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (enabled) {
      return launcher.isReadyToShoot();
    } else {
      return true;
    }
  }
}
