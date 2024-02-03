package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class SpikeUtils {
  public static double normalizeAngle(double angleDegrees) {
    angleDegrees %= 360;
    if (angleDegrees > 180.0d) {
      angleDegrees -= 360.0d;
    } else if (angleDegrees <= -180.0d) {
      angleDegrees += 360.0d;
    }

    return angleDegrees;
  }

  public static Rotation2d normalizeAngle(Rotation2d angleRotation2d) {
    return Rotation2d.fromDegrees(normalizeAngle(angleRotation2d.getDegrees()));
  }

  public static Rotation2d getDifference(Rotation2d alpha, Rotation2d beta) {
    double difference =
        SpikeUtils.normalizeAngle(beta).getDegrees()
            - SpikeUtils.normalizeAngle(alpha).getDegrees();

    if (difference > 180.0d) {
      difference -= 360.0d;
    } else if (difference < -180.0d) {
      difference += 360.0d;
    }

    return Rotation2d.fromDegrees(difference);
  }

  public static double getDifference(double alpha, double beta) {
    Rotation2d distance =
        getDifference(Rotation2d.fromDegrees(alpha), Rotation2d.fromDegrees(beta));
    return distance.getDegrees();
  }
}
