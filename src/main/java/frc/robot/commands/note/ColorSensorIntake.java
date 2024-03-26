package frc.robot.commands.note;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class ColorSensorIntake extends Command {
  // subsystems
  private final Intake intake;
  private final Launcher launcher;

  public ColorSensorIntake(Intake intake, Launcher launcher) {
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
    // If the color sensor senses a note, disable the intake
    if (launcher.isNoteDetected()) {
      intake.disableIntake();
      if (launcher.detectedNoteForSeconds() < 0.2) {
        intake.setVelocity(-1.0);
      } else {
        launcher.enableLauncher();
        intake.disableIntake();
      }
    } else {
      intake.enableIntake();
      launcher.disableLauncher();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
