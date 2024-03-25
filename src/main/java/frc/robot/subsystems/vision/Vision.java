package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.Led;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Vision extends SubsystemBase {

  private final VisionIO m_visionIO;
  private final VisionIOInputsAutoLogged m_visionInputs = new VisionIOInputsAutoLogged();
  private Drive m_Drive;
  private Led m_Led;

  public Vision(Drive drive, Led led) {
    // Initialize the ColorSensorIORevV3 object
    m_visionIO = new VisionIOLimelight("limelight");
    m_Drive = drive;
    m_Led = led;
  }

  public boolean seesTarget() {
    return m_visionInputs.seesTarget;
  }

  @Override
  public void periodic() {
    m_visionIO.updateInputs(m_visionInputs);
    if (m_visionInputs.seesTarget) {
      m_Led.setColor(0.71);
    } else {
      m_Led.setDefault();
    }
  }

  public Pose2d getBotPose() {
    return LimelightHelpers.getBotPose2d("limelight");
  }

  public Pose2d getAprilTagPose() {
    double[] position = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    double x = position[0];
    if (position[0] > 0) {
      x -= 3;
    } else {
      x += 3;
    }
    return new Pose2d(x, position[1], new Rotation2d(position[2]));
  }

  public Trajectory generateTrajectory(Pose2d end) {
    TrajectoryConfig config = new TrajectoryConfig(3, 3);
    config.setKinematics(m_Drive.getKinematics());
    return TrajectoryGenerator.generateTrajectory(m_Drive.getPose(), List.of(), end, config);
  }

  Command currentCommand = new SequentialCommandGroup();

  public Command runPath(Trajectory path) {
    if (m_visionInputs.seesTarget == false) {
      return new SequentialCommandGroup();
    }
    PIDController xController = new PIDController(0.1, 0, 0); // TODO: tune
    PIDController yController = new PIDController(0.1, 0, 0); // TODO: tune
    ProfiledPIDController tController =
        new ProfiledPIDController(
            0.1,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(10, 10)); // TODO: tune and change values
    HolonomicDriveController driveController =
        new HolonomicDriveController(xController, yController, tController);
    Consumer<SwerveModuleState[]> moduleStates = (states) -> {};
    moduleStates.accept(m_Drive.getStates());
    Supplier<Pose2d> pose = m_Drive::getPose;
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new SwerveControllerCommand(
                path, pose, m_Drive.getKinematics(), driveController, moduleStates));
    currentCommand = command;
    return command;
  }

  public void cancelCommand() {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
  }
}
