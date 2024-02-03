package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.constants.SpikePID;
import frc.robot.subsystems.drive.Drive;

public class MoveTo extends Command {
  private static final double TARGET_THRESHOLD_DEGREES = 2.0d;
  private static final double TARGET_THRESHOLD_POS_M = 0.1d;
  private static final double MAX_ACCELERATION_PERCENT = 0.3;

  private Drive drive;
  private Pose2d targetPose;

  private Pose2d lastPose;

  private double lastOutputX = 0.0;
  private double lastOutputY = 0.0;

  private double currentOutputX = 0.0;
  private double currentOutputY = 0.0;

  public MoveTo(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.targetPose = targetPose;

    lastPose = drive.getPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setTargetAngle(targetPose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get the pose error between the current pose and the robot
    Translation2d poseError = drive.getTranslationDifference(targetPose.getTranslation());

    // find change in acceleration
    Pose2d pose = drive.getPose();
    Translation2d changeInPosition =
        new Translation2d(pose.getX() - lastPose.getX(), pose.getY() - lastPose.getY());

    // apply the PID
    double outputX = SpikePID.getDriveOutput(poseError.getX(), 0, changeInPosition.getX());
    double outputY = SpikePID.getDriveOutput(poseError.getY(), 0, changeInPosition.getY());

    // Clamp the output so we don't move faster than the max speed
    outputX = MathUtil.clamp(outputX, -1.0, 1.0);
    outputY = MathUtil.clamp(outputY, -1.0, 1.0);

    // Calculate the acceleration
    double accelerationX = outputX - lastOutputX;
    double accelerationY = outputY - lastOutputY;

    accelerationX =
        MathUtil.clamp(accelerationX, -MAX_ACCELERATION_PERCENT, MAX_ACCELERATION_PERCENT);
    accelerationY =
        MathUtil.clamp(accelerationY, -MAX_ACCELERATION_PERCENT, MAX_ACCELERATION_PERCENT);

    // Update the current output
    currentOutputX += accelerationX;
    currentOutputY += accelerationY;

    // run field oriented drive using the outputX and outputY between -1.0 and 1.0
    drive.runFieldOriented(currentOutputX, currentOutputY);

    // Update all the lastOutputs
    lastOutputX = outputX;
    lastOutputY = outputY;
    lastPose = drive.getPose();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angleError = Math.abs(drive.getAngleError(targetPose.getRotation()).getDegrees());
    double distanceToTarget =
        Math.abs(
            drive
                .getTranslationDifference(targetPose.getTranslation())
                .getDistance(new Translation2d(0, 0)));

    return angleError < TARGET_THRESHOLD_DEGREES && distanceToTarget < TARGET_THRESHOLD_POS_M;
  }
}
