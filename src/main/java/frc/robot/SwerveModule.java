package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.util.CTREModuleState;
import frc.lib.util.Conversions;
import frc.lib.util.SPIKESwerveModConstants;
import frc.lib.util.SwerveConstants;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {

    public final int m_moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;
    private CTREConfigs m_configs;

    private final TalonFX m_angleMotor;
    private final TalonFX m_driveMotor;
    private final CANcoder m_angleEncoder;

    private PositionVoltage m_angleSetter = new PositionVoltage(0).withSlot(0);
    private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.Swerve.driveKS, SwerveConstants.Swerve.driveKV, SwerveConstants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SPIKESwerveModConstants moduleConstants) {
        m_configs = new CTREConfigs(moduleConstants);

        m_moduleNumber = moduleNumber;
        m_angleOffset  = moduleConstants.angleOffset;

        // Angle encoder config
        m_angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        // Angle motor config
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        // Drive motor config
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        m_lastAngle = getState().angle;
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        m_angleMotor.setNeutralMode(neutralMode);
        m_driveMotor.setNeutralMode(neutralMode);
    }

    public void stop() {
        m_driveMotor.set(0);
        m_angleMotor.set(0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (desiredState.speedMetersPerSecond < 0.2) {
            stop();
            return;
        }

        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.Swerve.maxSpeed;
            m_driveMotor.set(percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio);
            m_driveMotor.setControl(m_velocitySetter.withVelocity(velocity).withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.Swerve.maxSpeed * 0.01)) ? m_lastAngle : desiredState.angle; // Prevent rotating module if speed is less than 1%. Prevents jittering

        m_angleMotor.setControl(m_angleSetter.withPosition(Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.Swerve.angleGearRatio)));
        m_lastAngle = angle;
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition().getValue() * 360);
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_angleOffset.getDegrees(), SwerveConstants.Swerve.angleGearRatio);
        m_angleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(m_driveMotor.getPosition().getValue(), SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio),
            getAngle()
        );
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getPosition().getValue(), SwerveConstants.Swerve.angleGearRatio));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driveMotor.getPosition().getValue(), SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio),
            getAngle()
        );
    }

    public void configAngleEncoder() {
        m_angleEncoder.getConfigurator().apply(m_configs.swerveCANcoderConfig);
    }

    public void configAngleMotor() {
        m_angleMotor.getConfigurator().apply(m_configs.swerveAngleFXConfig);
        m_angleMotor.setNeutralMode(SwerveConstants.Swerve.driveNeutralMode);
        resetToAbsolute();
    }

    public void configDriveMotor() {
        m_driveMotor.getConfigurator().apply(m_configs.swerveDriveFXConfig);
        m_driveMotor.setNeutralMode(SwerveConstants.Swerve.driveNeutralMode);
        m_driveMotor.setPosition(0);
    }
}
