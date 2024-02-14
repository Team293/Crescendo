package frc.lib.constants;

public class SpikePID {
  public static double ANGLE_P = 5.0;
  public static double ANGLE_I = 0.1;
  public static double ANGLE_D = 0.0;

  public static double DRIVE_P = 0.005;
  public static double DRIVE_I = 0.0;
  public static double DRIVE_D = 0.0;

  public static double ROTATE_TO_P = 0.005;
  public static double ROTATE_TO_I = 0.0;
  public static double ROTATE_TO_D = 0.0;

  public static double DRIVE_TO_P = 0.005;
  public static double DRIVE_TO_I = 0.0;
  public static double DRIVE_TO_D = 0.0;

  public static double getRotationOutput(double error, double accumulation, double velocity) {
    return ROTATE_TO_P * error + ROTATE_TO_I * accumulation + ROTATE_TO_D * velocity;
  }

  public static double getDriveOutput(double error, double accumulation, double velocity) {
    return DRIVE_TO_P * error + DRIVE_TO_I * accumulation + DRIVE_TO_D * velocity;
  }
}
