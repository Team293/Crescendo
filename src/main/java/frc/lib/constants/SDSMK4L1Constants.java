package frc.lib.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public final class SDSMK4L1Constants {
  /** Wheel Characterization */
  public static final double wheelDiameter = Units.inchesToMeters(4.0);

  public static final double wheelCircumference = wheelDiameter * Math.PI;

  /** Gearing Characterization - MK4L2 Module */
  public static final double angleGearRatio = (12.8 / 1.0);

  public static final double driveGearRatio = (8.14 / 1.0);

  /** Motor Characterization Values */
  /* Divide SYSID values by 12 to convert from volts to percent output for CTRE */
  public static final double motorKS = 0.16d; // TODO: This must be tuned to specific robot

  public static final double motorKV = 0.113d;

  /** Angle motor PID */
  public static final double angleKP = 5.0d;

  public static final double angleKI = 0.01d;
  public static final double angleKD = 0.0d;

  /** Drive motor PID */
  public static final double driveKP = 0.1d;

  public static final double driveKI = 0.0d;
  public static final double driveKD = 0.0d;

  /** Inversion settings */
  public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

  public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
  public static final SensorDirectionValue canCoderSensorDirection =
      SensorDirectionValue.CounterClockwise_Positive;
}
