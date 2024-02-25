package frc.robot.commands.note;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class Launch extends Command {
  // state variables
  private boolean feeding = false;

  // subsystems
  private final Intake intake;
  private final Launcher launcher;

  public Launch(Intake intake, Launcher launcher) {
    this.intake = intake;
    this.launcher = launcher;

    addRequirements(intake, launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Disable the intake
    intake.disableIntake();
    // Enable the launcher
    launcher.enableLauncher();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if launcher is spun up
    if ((launcher.isLauncherReady()) && (false == feeding)) {
      feeding = true;
      // Turn on the intake to fire
      intake.enableIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the launcher
    intake.disableIntake();
    launcher.disableLauncher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
