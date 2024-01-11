package frc.robot.classes;

public class SmoothControl {
  public static final double K1 = 1.0d;
  public static final double K2 = 3.0d;

  public SmoothControl() {
  }

  // Modification of equation 13, Calculates the omegaDesired (in radians)
  // modified by velocity aggressivness given required turn rate
  public double computeTurnRate(Position2D currentPose, Position2D targetPose, double givenMaxVelocity, boolean inReverse) {
    double omegaDesired = 0.0d; 
    double poseHeading = currentPose.getHeadingRadians(); // Convert heading to radians
    double targetHeading = targetPose.getHeadingRadians();
    double maxVelocity = givenMaxVelocity;

    // Are we going in reverse? 
    if(inReverse) {
      // Subtract PI to the current and target headings to "trick" smooth control into thinking we're facing forwards
      poseHeading += Math.PI;
      targetHeading += Math.PI;

      // The given velocity will be negative, we must make it positive
      maxVelocity *= -1.0d;
    }

    // Limit pose heading to be within -Pi and Pi
    poseHeading = limitRadians(poseHeading); // poseHeading is now radians

    // With the velocity and robot heading set appropriately, get the range
    // and the vector orientation that runs from the robot to the target
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();
    double range = getRange(targetPose, currentPose); // distance in feet
    double r_angle = Math.atan2(dy, dx); // vector heading in radians

    if (range != 0.0) {
      // Compute the angle between this vector and the desired orientation at the
      // target
      double thetaT = targetHeading - r_angle;
      thetaT = limitRadians(thetaT); // bound this between -PI to PI

    // Compute the angle between current robot heading and the vector from
    // the robot to the target
    double del_r = poseHeading - r_angle;
    del_r = limitRadians(del_r); // bound this between -PI to PI

    // Calculate k (equation 14)
    double k = calculateK(range, thetaT, del_r);

      // All set, now the equation for the angular rate!
      omegaDesired = vGivenK(k, maxVelocity) * k;
    } 

    return (omegaDesired);
  }

  // Equation 14, Calculates the required turn rate
  private double calculateK(double range, double theta, double delta) {
    double retval = 0.0d;
    if(0.0d != range) {
      retval = (-(1 / range) * (K2 * (delta - Math.atan(-K1 * theta)) +
      Math.sin(delta) * (1.0 + (K1 / (1.0 + Math.pow((K1 * theta), 2))))));
    }
    return (retval);
  }

  // Equation 15, Calculates how close we are to maximum velocity based off of
  // required turn rate
  private double vGivenK(double k, double vMax) {
    double beta = 0.4d;
    double lambda = 2;
    return (vMax / (1 + Math.abs(Math.pow((beta * k), lambda))));
  }

  public double limitRadians(double radians) {
    double retval = radians;

    while (retval > Math.PI) {
      retval -= 2 * Math.PI;
    }

    while (retval < -Math.PI) {
      retval += 2 * Math.PI;
    }

    return retval;
  }

  public double getRange(Position2D poseA, Position2D poseB) {
    double dx = poseB.getX() - poseA.getX();
    double dy = poseB.getY() - poseA.getY();
    return (Math.sqrt(dx * dx + dy * dy)); // distance in feet
  }
}