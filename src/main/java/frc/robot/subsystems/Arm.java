package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    /* Constants */
    public final int PID_CONFIG_TIMEOUT_MS = 10;
    public final int PIVOT_TALON_FX_CAN_ID = 4;
    public final int EXTENDER_TALON_FX_CAN_ID = 5;
    public final int CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS = 4000;
    public final double PIVOT_KF = 0.0112d;
    public final double PIVOT_KP = 0.3d;
    public final double PIVOT_KI = 0.0d;
    public final double PIVOT_KD = 3d;
    public final double PIVOT_MAX_VELOCITY = 3000d;
    public final double PIVOT_MAX_ACCELERATION = 2000d;
    public final double MOTOR_NEUTRAL_DEADBAND = 0.001d;
    public final double PERIODIC_RUNS_PER_SECOND = 50.0d;
    public final double ARM_X_DELTA_MODIFIER = 6.0 / PERIODIC_RUNS_PER_SECOND;
    public final double ARM_Y_DELTA_MODIFIER = 6.0 / PERIODIC_RUNS_PER_SECOND;
    public final double ARM_PIVOT_X_INCHES = 17.0d;
    public final double ARM_PIVOT_Y_INCHES = 54.0d;
    public final double CALIBRATION_MOTOR_SPEED = 0.1;
    public final double SCORE_HYBRID_X_INCHES = 20.0d; // TODO: Low: Find the value for how far the extension motor has
                                                       // to extend in inches.
    public final double SCORE_HYBRID_ANGLE = 42.0d;
    public final double SCORE_MID_X_INCHES = 40.0d; // TODO: Mid: Find the value for how far the extension motor has to
                                                    // extend in inches.
    public final double SCORE_MID_ANGLE = 87.0d;
    public final double SCORE_HIGH_X_INCHES = 60.0; // TODO: High: Find the value for how far the extension motor has to
                                                    // extend in inches.
    public final double SCORE_HIGH_ANGLE = 106.0d;
    public final double SUBSTATION_PICKUP_ANGLE = 104.0d;
    public final double SUBSTATION_PICKUP_X_INCHES = 10.0d; // TODO: High: Find the value for how far the extension
                                                            // motor has to extend in inches.
    // Angles are in DEGREES
    public final double MIN_ANGLE = 0;
    public final double MAX_ANGLE = 110;
    // Arm shoulder Y inches is between pivot point and ground
    public final double ENCODER_UNITS_PER_REVOLUTION = 2048.0d;
    public final double PIVOT_ENCODER_UNITS_PER_DEGREE = ENCODER_UNITS_PER_REVOLUTION / DEGREES_PER_REVOLUTION;
    public final double EXTENDER_ENCODER_UNITS_PER_INCH = 5.0d; // NEED TO FIND THIS
    // Gear ratios
    public final double MOTOR_ROTATIONS_PER_ENCODER_UNIT = 1.0d / ENCODER_UNITS_PER_REVOLUTION;
    public final double PIVOT_GEARBOX_RATIO = 36.00d / 1.0d;
    public final double PULLEY_RATIO = 2.0d / 1.0d;
    public final double EXTENSION_RATIO = 4.0d / 1.0d;

    public final double EXTENDER_KF = 0.0d;
    public final double EXTENDER_KP = 0.1d;
    public final double EXTENDER_KI = 0.001d;
    public final double EXTENDER_KD = 1d;
    public final double EXTENDER_MAX_VELOCITY = 30000d;
    public final double EXTENDER_MAX_ACCELERATION = 30000d;

    /* Members */
    private WPI_TalonFX pivotTalonFX;
    private WPI_TalonFX extenderTalonFX;
    private double x;
    private double y;

    public static final double POSITION_KF = 0.0d;
    public static final double POSITION_KP = 0.0d;
    public static final double POSITION_KI = 0.0d;
    public static final double POSITION_KD = 0.0d;
    public static final double DEGREES_PER_REVOLUTION = 360;

    // Gear ratios
    public Arm() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        pivotTalonFX = new WPI_TalonFX(PIVOT_TALON_FX_CAN_ID);
        extenderTalonFX = new WPI_TalonFX(EXTENDER_TALON_FX_CAN_ID);

        // Clears motor errors
        pivotTalonFX.clearStickyFaults();
        extenderTalonFX.clearStickyFaults();

        // Set factory defaults for onboard PID
        pivotTalonFX.configFactoryDefault();
        extenderTalonFX.configFactoryDefault();

        pivotTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS);
        extenderTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS);

        pivotTalonFX.setInverted(true);
        pivotTalonFX.setSensorPhase(true);
        extenderTalonFX.setInverted(false);
        extenderTalonFX.setSensorPhase(false);

        // Configure Position PID
        pivotTalonFX.config_kF(0, PIVOT_KF, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.config_kP(0, PIVOT_KP, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.config_kI(0, PIVOT_KI, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.config_IntegralZone(0, 1000);
        pivotTalonFX.config_kD(0, PIVOT_KD, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.configMotionCruiseVelocity(PIVOT_MAX_VELOCITY, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.configMotionAcceleration(PIVOT_MAX_ACCELERATION, PID_CONFIG_TIMEOUT_MS);

        extenderTalonFX.config_kF(0, EXTENDER_KF, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.config_kP(0, EXTENDER_KP, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.config_kI(0, EXTENDER_KI, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.config_IntegralZone(0, 100);
        extenderTalonFX.config_kD(0, EXTENDER_KD, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.configMotionCruiseVelocity(EXTENDER_MAX_VELOCITY, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.configMotionAcceleration(EXTENDER_MAX_ACCELERATION, PID_CONFIG_TIMEOUT_MS);

        extenderTalonFX.setNeutralMode(NeutralMode.Brake);
        pivotTalonFX.setNeutralMode(NeutralMode.Brake);

        extenderTalonFX.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);
        pivotTalonFX.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);
    }

    @Override
    public void periodic() {
        moveToPosition(x, y);
    }

    /**
     * Rotate the arm to a specific angle.
     * 
     * @param angle - the angle in degrees
     */
    public void rotateTo(double angle) {
        angle = Math.max(Math.min(angle, MAX_ANGLE), MIN_ANGLE);
        double encoderUnits = angle * PIVOT_ENCODER_UNITS_PER_DEGREE;
        pivotTalonFX.set(TalonFXControlMode.MotionMagic, encoderUnits);
    }

    /**
     * Extends arm to a given amount of inches.
     * 
     * @param length - the length to extend/retract to in inches
     */
    public void extendTo(double length) {
        double encoderUnits = length * EXTENDER_ENCODER_UNITS_PER_INCH;
        extenderTalonFX.set(TalonFXControlMode.MotionMagic, encoderUnits);
    }

    /**
     * Moves the arm to a given position in space relative to the base.
     * 
     * @param targetX - position in inches
     * @param targetY - position in inches
     */
    public void moveToPosition(double targetX, double targetY) {
        double xDist = targetX - ARM_PIVOT_X_INCHES;
        double yDist = targetY - ARM_PIVOT_Y_INCHES;
        double length = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
        double theta = Math.atan2(yDist, xDist);

        rotateTo(theta);
        extendTo(length);
    }

    /**
     * Sets the internal position of the arm.
     * 
     * @param xPos - position in inches
     * @param yPos - position in inches
     */
    public void setPosition(double xPos, double yPos) {
        x = xPos;
        y = yPos;
    }

    public void adjustPosition(double xPercentage, double yPercentage) {
        x += ARM_X_DELTA_MODIFIER * xPercentage;
        y += ARM_Y_DELTA_MODIFIER * yPercentage;
    }

    public double encoderToDegrees(double encoderUnits) {
        return encoderUnits / ENCODER_UNITS_PER_REVOLUTION * DEGREES_PER_REVOLUTION;
    }

    public void pivotZero() {
        pivotTalonFX.setSelectedSensorPosition(0);
    }

    public void extenderZero() {
        extenderTalonFX.setSelectedSensorPosition(0);
    }

    public void startPivotCalibration() {
        pivotTalonFX.set(-CALIBRATION_MOTOR_SPEED);
    }

    public void startExtenderCalibration() {
        extenderTalonFX.set(-CALIBRATION_MOTOR_SPEED);
    }

    public boolean checkCalibration() {
        boolean done = true;
        if (pivotTalonFX.isRevLimitSwitchClosed() == 1) {
            pivotTalonFX.set(0);
        } else {
            done = false;
        }
        if (extenderTalonFX.isRevLimitSwitchClosed() == 1) {
            extenderTalonFX.set(0);
        } else {
            done = false;
        }
        return done;
    }
}
