package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class RotateTo extends Command {
  private Drive drive;
  private Rotation2d targetRotation2d;
  private double targetTurningVelocity;
  private double angleError;

  public RotateTo(Drive drive, Rotation2d targetRotation2d) {
    this.drive = drive;
    this.targetRotation2d = targetRotation2d;
    this.angleError = 0d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleError = getDistance(drive.getRotation(), targetRotation2d);
    targetTurningVelocity = angleError / 360.0d;

    Logger.recordOutput("TurningError", angleError);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0d,
            0.0d,
            MathUtil.clamp(targetTurningVelocity, -1.0, 1.0) * drive.getMaxAngularSpeedRadPerSec(),
            drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public static double getDistance(Rotation2d alpha, Rotation2d beta) {
    double phi = beta.getDegrees() - alpha.getDegrees();
    double difference = (phi + 180.0) % 360.0 - 180.0;
    return difference;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(angleError) < 5) {
      return true;
    }
    return false;
  }
}
