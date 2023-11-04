package frc.lib.util;

import edu.wpi.first.math.MathUtil;

public final class SPIKE293Utils {

    public static final double MAX_ENCODER_VELOCITY = 20743.0d;

    public static final double WHEEL_CIRCUMFERENCE_FEET = (4.0d / 12.0d) * Math.PI; // Wheel radius 4 in, converting
                                                                                        // to feet
    public static final double SECONDS_TO_DECISEC = 1.0d / 10.0d;
    public static final double DECISEC_TO_SECONDS = 10.0d / 1.0d;
    public static final double GEARBOX_RATIO_TO_ONE = 9.52d;

    public static final int ENCODER_COUNTS_PER_REVOLUTION = 2048;
    public static final int ENCODER_UNITS_PER_REVOLUTION = ENCODER_COUNTS_PER_REVOLUTION; // Edges per Rotation

    public static final double GEAR_RATIO = 1.0d;
    public static final double MINUTES_TO_DECISECONDS = 600.0d;


    private SPIKE293Utils() {
        throw new AssertionError("utility class");
    }

    /**
     * 
     * @param input
     * @param deadband
     * @return deadband applied output
     */
    public static double applyDeadband(double input, double deadband) {
        double retval = input;
        if (Math.abs(input) >= deadband) {
            retval = (Math.abs(input) - deadband) / (1 - deadband);
            // Check if value is supposed to be negative
            if (input < 0) {
                retval = -1 * retval;
            }
        } else {
            retval = 0;
        }

        return retval;
    }

    /**
     * Converts from encoder edges per 100 milliseconds to feet per second.
     * 
     * @return Drivetrain velocity in ft/s from encoder units
     */
    public static double controllerVelocityToFeetPerSec(double encoderUnits) {
        return controllerUnitsToFeet(encoderUnits) * DECISEC_TO_SECONDS;
    }

    /**
     * Converts from feet per second to encoder edges per 100 milliseconds.
     * 
     * @param Speed in ft/s
     * 
     * @return Drivetrain velocity in encoder units(edges per 100 milliseconds)
     */
    public static double feetPerSecToControllerVelocity(double feetPerSec) {
        return ((feetPerSec * GEARBOX_RATIO_TO_ONE * ENCODER_UNITS_PER_REVOLUTION) / (WHEEL_CIRCUMFERENCE_FEET))
                * SECONDS_TO_DECISEC;
    }

    /**
     * 
     * @param Encoder Units
     * @return feet from encoder units for drivetrain
     */
    public static double controllerUnitsToFeet(double encoderUnits) {
        return (encoderUnits * WHEEL_CIRCUMFERENCE_FEET) / (GEARBOX_RATIO_TO_ONE * ENCODER_UNITS_PER_REVOLUTION);
    }

    /**
     * 
     * @param feet
     * @return encoder units from feet for drivetrain
     */
    public static double feetToControllerUnits(double feet) {
        double controllerUnits = 0.0d;
        controllerUnits = ((feet * GEARBOX_RATIO_TO_ONE * ENCODER_UNITS_PER_REVOLUTION) / (WHEEL_CIRCUMFERENCE_FEET));
        return controllerUnits;
    }

    /**
     * Converts percentage to encoder velocity
     * 
     * @param percentage
     * @return velocity in encoder units(edges per 100 milliseconds) for drivetrain
     */
    public static double percentageToControllerVelocity(double percentage) {
        MathUtil.clamp(percentage, -1.0d, 1.0d);
        return (percentage * MAX_ENCODER_VELOCITY);
    }

    /**
     * Converts RPMs to encoder units per desiseconds (100ms)
     * 
     * @param rpm
     * @return velocity in encoder units (edges per 100) of shooter wheel
     */
    public static double convertRPMToControllerVelocity(double rpm) {
        return (rpm * GEAR_RATIO * ENCODER_UNITS_PER_REVOLUTION) / MINUTES_TO_DECISECONDS;
    }

    /**
     * Converts Encoder units per desiseconds (100ms) to RPMs
     * 
     * @param velocity (encoder units per 100 milliseconds)
     * @return rpm of shooter wheel
     */
    public static double convertControllerVelocityToRPM(double velocity) {
        return (velocity * MINUTES_TO_DECISECONDS) / (ENCODER_UNITS_PER_REVOLUTION * GEAR_RATIO);
    }
}
