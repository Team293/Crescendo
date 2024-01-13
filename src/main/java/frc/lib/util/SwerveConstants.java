package frc.lib.util;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final InvertedValue canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleSupplyCurrentLimit = 25;
        public static final int angleSupplyCurrentThreshold = 40;
        public static final double angleSupplyTimeThreshold = 0.1;
        public static final boolean angleSupplyCurrentLimitEnable = true;

        public static final int driveSupplyCurrentLimit = 35;
        public static final int driveSupplyCurrentThreshold = 60;
        public static final double driveSupplyTimeThreshold = 0.1;
        public static final boolean driveSupplyCurrentLimitEnable = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.005; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        // Updated encoder offsets, need to convert to degrees and set as angleOffset
        // Front left: -6368
        //back left: 19290
        //front right: -6886
        // back right: -6484

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(44.9 + 90.0);
            
            public static final double angleKP = COTSFalconSwerveConstants.SDSMK4(0).angleKP;
            public static final double angleKI = COTSFalconSwerveConstants.SDSMK4(0).angleKI;
            public static final double angleKD = COTSFalconSwerveConstants.SDSMK4(0).angleKD;

            public static final SPIKESwerveModConstants constants = 
                new SPIKESwerveModConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-101.3 + 90.0);

            public static final double angleKP = COTSFalconSwerveConstants.SDSMK4(0).angleKP;
            public static final double angleKI = COTSFalconSwerveConstants.SDSMK4(0).angleKI;
            public static final double angleKD = COTSFalconSwerveConstants.SDSMK4(0).angleKD;

            public static final SPIKESwerveModConstants constants = 
                new SPIKESwerveModConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(328.1 + 90.0);

            public static final double angleKP = COTSFalconSwerveConstants.SDSMK4(0).angleKP;
            public static final double angleKI = COTSFalconSwerveConstants.SDSMK4(0).angleKI;
            public static final double angleKD = COTSFalconSwerveConstants.SDSMK4(0).angleKD;

            public static final SPIKESwerveModConstants constants = 
                new SPIKESwerveModConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-43.8 + 90.0);

            public static final double angleKP = COTSFalconSwerveConstants.SDSMK4(0).angleKP;
            public static final double angleKI = COTSFalconSwerveConstants.SDSMK4(0).angleKI;
            public static final double angleKD = COTSFalconSwerveConstants.SDSMK4(0).angleKD;

            public static final SPIKESwerveModConstants constants = 
                new SPIKESwerveModConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}