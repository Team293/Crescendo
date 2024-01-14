package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveConstants;
import frc.lib.util.SPIKESwerveModConstants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCANcoderConfig;

    public CTREConfigs(SPIKESwerveModConstants moduleConstants){
        /* Swerve Angle Motor Configurations */
        swerveAngleFXConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        Slot0Configs anglePIDConfigs = new Slot0Configs();

        /* Angle - Set Current Limits */
        angleSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.Swerve.angleSupplyCurrentLimitEnable;
        angleSupplyLimit.SupplyCurrentLimit = SwerveConstants.Swerve.angleSupplyCurrentLimit;
        angleSupplyLimit.SupplyCurrentThreshold = SwerveConstants.Swerve.angleSupplyCurrentThreshold;
        angleSupplyLimit.SupplyTimeThreshold = SwerveConstants.Swerve.angleSupplyTimeThreshold;

        /* Angle - Set PID */
        anglePIDConfigs.kP = SwerveConstants.Swerve.angleKP;
        anglePIDConfigs.kI = SwerveConstants.Swerve.angleKI;
        anglePIDConfigs.kD = SwerveConstants.Swerve.angleKD;

        /* Angle - Set Inverted*/
        swerveAngleFXConfig.MotorOutput.Inverted = COTSFalconSwerveConstants.SDSMK4(SwerveConstants.Swerve.driveGearRatio).angleMotorInvert;

        /* Angle - Set Continuous Wrap Configs */
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = COTSFalconSwerveConstants.SDSMK4(SwerveConstants.Swerve.driveGearRatio).angleGearRatio; // Set the gear ratio of the motor to the mechanism

        /* Angle - Set Remote Sensor Configs */
        // We should consider whether or not to use this. We would have to see which way
        swerveAngleFXConfig.Feedback.FeedbackRemoteSensorID = moduleConstants.cancoderID;
        swerveAngleFXConfig.Feedback.RotorToSensorRatio = 1.0 / SwerveConstants.Swerve.angleGearRatio; // Set the gear ratio of the motor to the sensor (1:12.8) (12.8 rotations of the motor = 1 rotation of the CANcoder)

        /* Swerve Drive Motor Configuration */
        swerveDriveFXConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        Slot0Configs drivePIDConfigs = new Slot0Configs();

        /* Drive - Set Current Limits */
        driveSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.Swerve.driveSupplyCurrentLimitEnable;
        driveSupplyLimit.SupplyCurrentLimit = SwerveConstants.Swerve.driveSupplyCurrentLimit;
        driveSupplyLimit.SupplyCurrentThreshold = SwerveConstants.Swerve.driveSupplyCurrentThreshold;
        driveSupplyLimit.SupplyTimeThreshold = SwerveConstants.Swerve.driveSupplyTimeThreshold;

        /* Drive - Set PID */
        drivePIDConfigs.kP = SwerveConstants.Swerve.driveKP;
        drivePIDConfigs.kI = SwerveConstants.Swerve.driveKI;
        drivePIDConfigs.kD = SwerveConstants.Swerve.driveKD;

        drivePIDConfigs.kS = SwerveConstants.Swerve.driveKS;
        drivePIDConfigs.kV = SwerveConstants.Swerve.driveKV;
        drivePIDConfigs.kA = SwerveConstants.Swerve.driveKA;

        /* Drive - Set Inverted*/
        swerveDriveFXConfig.MotorOutput.Inverted = COTSFalconSwerveConstants.SDSMK4(SwerveConstants.Swerve.driveGearRatio).driveMotorInvert;

        /* Swerve CANcoder Configuration */
        swerveCANcoderConfig = new CANcoderConfiguration();
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // swerveCANcoderConfig.MagnetSensor.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCANcoderConfig.MagnetSensor.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Apply Configs */
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;
        swerveAngleFXConfig.Slot0 = anglePIDConfigs;

        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;
        swerveDriveFXConfig.Slot0 = drivePIDConfigs;
    }
}