package frc.robot.classes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class SpikeController {
    private final XboxController m_controller;
    private final double m_deadband;
    private final String m_name;

    public SpikeController(XboxController controller, double deadband, String name) {
        m_controller = controller;
        m_name = name;
        m_deadband = MathUtil.clamp(deadband, 0.0d, 1.0d);
    }

    public double deadband(double value, double deadband) {
        return Math.abs(value) >= deadband ? value : 0.0d;
    }

    public Rotation2d getLeftDirection() {
        return new Rotation2d(m_controller.getLeftX(), m_controller.getLeftY());
    }

    public Rotation2d getRightDirection() {
        return new Rotation2d(m_controller.getRightX(), m_controller.getRightY());
    }

    public double getLeftMagnitude() {
        double magnitude = Math.sqrt(Math.pow(getLeftX(), 2) + Math.pow(getLeftY(), 2));
        return magnitude;
    }

    public double getRightMagnitude() {
        double magnitude = Math.sqrt(Math.pow(getRightX(), 2) + Math.pow(getRightY(), 2));
        return magnitude;
    }

    public double getLeftX() {
        return deadband(m_controller.getLeftX(), m_deadband);
    }

    public double getLeftY() {
        return deadband(m_controller.getLeftY(), m_deadband);
    }

    public double getRightX() {
        return deadband(m_controller.getRightX(), m_deadband);
    }

    public double getRightY() {
        return deadband(m_controller.getRightY(), m_deadband);
    }

    public double getLeftTrigger() {
        return m_controller.getLeftTriggerAxis();
    }

    public double getRightTrigger() {
        return m_controller.getRightTriggerAxis();
    }

    public boolean getLeftBumper() {
        return m_controller.getLeftBumper();
    }

    public boolean getRightBumper() {
        return m_controller.getRightBumper();
    }

    public boolean getAButton() {
        return m_controller.getAButton();
    }

    public boolean getBButton() {
        return m_controller.getBButton();
    }

    public boolean getXButton() {
        return m_controller.getXButton();
    }

    public boolean getYButton() {
        return m_controller.getYButton();
    }
}
