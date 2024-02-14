package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class RotateTo extends Command {
  private static final double TARGET_THRESHOLD_DEGREES = 2.0d;

  private Drive drive;
  private Rotation2d targetAngle;

  public RotateTo(Drive drive, Rotation2d targetRotation2d) {
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setTargetAngle(targetAngle.getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.runFieldOriented(0.0d, 0.0d);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drive.getAngleError(targetAngle).getDegrees()) < TARGET_THRESHOLD_DEGREES;
  }
}
