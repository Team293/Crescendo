package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SPIKESwerveModConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    public final double angleKP;
    public final double angleKI;
    public final double angleKD;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SPIKESwerveModConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, double angleKP, double angleKI, double angleKD) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;

        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
    }
}