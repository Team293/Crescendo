package frc.robot.commands.note;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class FeedNoteToLauncher extends Command {
  private static final double INTAKE_FEED_SPEED_RPS = 10.0;

  // state variables
  private boolean feeding = false;
  private boolean feedComplete = false;

  // subsystems
  private final Intake intake;
  private final Launcher launcher;

  public FeedNoteToLauncher(Intake intake, Launcher launcher) {
    this.intake = intake;
    this.launcher = launcher;

    addRequirements(intake, launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcher.enableLauncher();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if not feeding and the launcher is ready, start the intake
    intake.setVelocity(INTAKE_FEED_SPEED_RPS);
    if (launcher.isReadyToShoot()) {
      feeding = true;
    }

    // if we are feeding and the launcher is not ready this means the
    // launcher has received resistance from the note and slowed down
    if (feeding && !launcher.isReadyToShoot()) {
      feedComplete = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the launcher
    launcher.disableLauncher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the note has been launched, wait to make sure it has cleared the launcher
    return feedComplete;
  }
}
