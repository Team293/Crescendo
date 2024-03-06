package frc.robot.commands.note;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import java.util.function.BooleanSupplier;

public class Launch extends Command {
  // state variables
  private boolean feeding = false;
  private boolean complete = false;

  private Timer endBuffer = new Timer();
  private Timer totalTime = new Timer();

  // subsystems
  private final Intake intake;
  private final Launcher launcher;

  public Launch(Intake intake, Launcher launcher) {
    this.intake = intake;
    this.launcher = launcher;

    addRequirements(intake, launcher);
  }

  public Launch(Intake intake, Launcher launcher, BooleanSupplier cancelCommand) {
    this.intake = intake;
    this.launcher = launcher;

    onlyWhile(cancelCommand);

    addRequirements(intake, launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Disable the intake
    intake.disableIntake();
    // Enable the launcher
    launcher.enableLauncher();
    totalTime.reset();
    totalTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if launcher is spun up
    if (feeding && !launcher.isReadyToShoot()) {
      complete = true;
    }

    if (launcher.isReadyToShoot()) {
      intake.setVelocity(20.0);
      feeding = true;
    }

    if (!complete) {
      endBuffer.reset();
      endBuffer.start();
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
    return endBuffer.hasElapsed(0.2) || totalTime.hasElapsed(1.5);
  }
}
