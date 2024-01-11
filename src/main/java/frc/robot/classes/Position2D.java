package frc.robot.classes;

public class Position2D {
  private double m_x;
  private double m_y;
  private double m_heading; // In radians

  public Position2D(double x, double y, double heading) {
    m_x = x;
    m_y = y;
    m_heading = heading;
  }

  public void setX(double x) {
    m_x = x;
  }

  public void setY(double y) {
    m_y = y;
  }

  public void setHeadingDegrees(double heading) {
    m_heading = Math.toRadians(heading);
  }

  public void setHeadingRadians(double heading) {
    m_heading = heading;
  }

  public double getX() {
    return m_x;
  }

  public double getY() {
    return m_y;
  }

  public double getHeadingDegrees() {
    return Math.toDegrees(m_heading);
  }

  public double getHeadingRadians() {
    return m_heading;
  }
}
