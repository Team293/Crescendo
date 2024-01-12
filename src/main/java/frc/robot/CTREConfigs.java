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
        anglePIDConfigs.kP = moduleConstants.angleKP;
        anglePIDConfigs.kI = moduleConstants.angleKI;
        anglePIDConfigs.kD = moduleConstants.angleKD; // SwerveConstants.Swerve.angleKD

        /* Angle - Set Inverted*/
        swerveAngleFXConfig.MotorOutput.Inverted = COTSFalconSwerveConstants.SDSMK4(SwerveConstants.Swerve.driveGearRatio).angleMotorInvert;

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