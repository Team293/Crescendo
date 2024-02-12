package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private static final double ANGULAR_INTEGRAL_LIMIT = 5;

  private final VisionIO m_visionIO;
  private final VisionIOInputsAutoLogged m_visionInputs = new VisionIOInputsAutoLogged();
  private double m_angularP;
  private double m_angularI;
  private double m_angularD;
  private double m_angularLastError;
  private double m_angularError;
  private double m_angularChange;
  private double m_angularIntegralError;
  private double m_angularVelOutput;

  public Vision() {
    m_angularP = 0.005;
    m_angularI = 0.0;
    m_angularD = 0.0001;
    // Initialize the ColorSensorIORevV3 object
    m_visionIO = new VisionIOLimelight("limelight");
  }

  public boolean seesTarget() {
    return m_visionInputs.seesTarget;
  }

  public double getTX() {
    return m_visionInputs.tX;
  }

  public double getTY() {
    return m_visionInputs.tY;
  }

  public void resetError() {
    m_angularChange = 0.0;
    m_angularIntegralError = 0.0;
    m_angularError = 0.0;
  }

  public double getDesiredAngle() {
    return m_angularVelOutput;
  }

  @Override
  public void periodic() {
    m_visionIO.updateInputs(m_visionInputs);

    // Save the last error
    m_angularLastError = m_angularError;

    // Update the error
    //tX is in degrees from center
    // Divide by horizontal FOV / 2 to get a value between -1.0 and 1.0
    m_angularError = getTX() / (60.6 / 2.0);

    // Calculatge the angular change
    m_angularChange = m_angularError - m_angularLastError;

    if (Math.abs(m_angularError) < ANGULAR_INTEGRAL_LIMIT) {
      m_angularIntegralError += m_angularError;
    }

    //  Angular align
    m_angularVelOutput =
        (m_angularP * m_angularError)
            + (m_angularI * m_angularIntegralError)
            + (m_angularD * m_angularChange);
            
    //Error of 10% demand
    if (Math.abs(m_angularError) < 0.1) {
      /* Found the target */
      m_angularVelOutput = 0.0d;
      m_angularIntegralError = 0.0d;
    }

    Logger.processInputs("Vision", m_visionInputs);
  }
}
