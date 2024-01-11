package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class SequentialAutoCommand extends SequentialCommandGroup {
    public static enum StartPositions {
        DONT_MOVE,
        DRIVE_BACKWARD,
        LEFT_SIDE_SCORE,
        CENTER_ENGAGE,
        RIGHT_SIDE_SCORE,
		SCORE_DONT_MOVE,
		SCORE_AND_ENGAGE
	}

	private StartPositions m_startPosition;
	private Drivetrain m_drivetrain;

	public SequentialAutoCommand(Drivetrain drivetrain, StartPositions startPosition) {
		m_drivetrain = drivetrain;
		m_startPosition = startPosition;

		// Put in commands here
	}
}