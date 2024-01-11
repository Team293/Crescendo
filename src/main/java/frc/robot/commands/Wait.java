package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    private double m_waitTime;
    private double m_startTime;

    /**
     * Creates a new Wait.
     * @param waitTime The time to wait in seconds
     */
    public Wait(double waitTime) {
        m_waitTime = waitTime;

        m_waitTime *= 1000;
        m_startTime = 0;
    }

    @Override
    public void initialize() {
        m_startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    /**
     * Returns true when the wait time has elapsed
     * @return true when the wait time has elapsed
     */
    @Override
    public boolean isFinished() {
        return ((System.currentTimeMillis() - m_startTime) >= m_waitTime);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
