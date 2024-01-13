package frc.lib.util;

public class Conversions {

    /**
     * @param rotation CANCoder Rotation (0.0-1.0)
     * @param gearRatio Gear Ratio between CANcoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double rotation, double gearRatio) {
        return (rotation * 360.0) / gearRatio;
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANcoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return (degrees / 360.0) * gearRatio;
    }

    /**
     * @param rotations Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconRotationsToDegrees(double rotations, double gearRatio) {
        return (rotations * 360.0) / gearRatio;
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon rotations
     */
    public static double degreesToFalconRotations(double degrees, double gearRatio) {
        return (degrees / 360.0) * gearRatio;
    }

    /**
     * @param velocityRPS Rotations per second
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Velocity of mechanism (in rotations/minute)
     */
    public static double falconRPSToRPM(double velocityRPS, double gearRatio) {
        final double secondsPerMinute = 60.0;
        double motorRPM = velocityRPS * secondsPerMinute;        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param mechRPM RPM of the mechanism (not the motor)
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon motor velocity (in rotations/second)
     */
    public static double RPMToFalconRPS(double mechRPM, double gearRatio) {
        final double minutesPerSecond = 1.0 / 60.0;
        double motorRPM = mechRPM * gearRatio;
        double velocityRPS = motorRPM * minutesPerSecond;
        return velocityRPS;
    }

    /**
     * @param motorRPS Falcon motor velocity (in rotations/second)
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Velocity of wheel (in meters/second)
     */
    public static double falconRPSToMPS(double motorRPS, double circumference, double gearRatio){
        final double minutesPerSecond = 1.0 / 60.0;
        double wheelRPM = falconRPSToRPM(motorRPS, gearRatio);
        double wheelMPS = (wheelRPM * circumference) * minutesPerSecond;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS (of the wheel)
     * @param circumference Circumference of wheel
     * @param gearRatio Gear Ratio between Falcon and wheel (set to 1 for Falcon MPS)
     * @return Falcon velocity (in rotations/second)
     */
    public static double MPSToFalcon(double mechMPS, double circumference, double gearRatio){
        final double secondsPerMinute = 60.0;
        double wheelRPM = (mechMPS * secondsPerMinute) / circumference;
        double motorRPS = RPMToFalconRPS(wheelRPM, gearRatio);
        return motorRPS;
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio));
    }

    // /**
    //  * @param meters Meters
    //  * @param circumference Circumference of Wheel
    //  * @param gearRatio Gear Ratio between Falcon and Wheel
    //  * @return Falcon rotations
    //  */
    // public static double MetersToFalconRotations(double meters, double circumference, double gearRatio){
    //     return meters / (circumference / (gearRatio));
    // }
}