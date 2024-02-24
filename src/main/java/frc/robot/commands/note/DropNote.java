package frc.robot.commands.note;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class DropNote extends Command {
  private static final double INTAKE_FEED_SPEED_RPS = -20.0;
  private static final double LENGTH_TIME_SEC = 0.3;

  // subsystems
  private final Intake intake;
  private final Timer timer = new Timer();

  public DropNote(Intake intake, BooleanSupplier cancelCommand) {
    this.intake = intake;
    // onlyWhile(() -> !cancelCommand.getAsBoolean());

    addRequirements(intake);
  }

  public DropNote(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setVelocity(INTAKE_FEED_SPEED_RPS);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      intake.disableIntake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(LENGTH_TIME_SEC);
  }
}
