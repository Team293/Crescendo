package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CTREModuleState;
import frc.lib.util.Conversions;
import frc.lib.util.SPIKESwerveModConstants;
import frc.lib.util.SwerveConstants;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {

    public final int m_moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;
    private CTREConfigs m_configs;
    public double m_targetRotations;

    private final TalonFX m_angleMotor;
    private final TalonFX m_driveMotor;
    private final CANcoder m_angleEncoder;

    private PositionVoltage m_angleSetter = new PositionVoltage(0).withSlot(0);
    private VelocityVoltage m_velocitySetter = new VelocityVoltage(0).withSlot(0);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.Swerve.driveKS, SwerveConstants.Swerve.driveKV, SwerveConstants.Swerve.driveKA);

    /**
     * Creates a new SwerveModule
     * @param moduleNumber The module number of the module
     * @param moduleConstants The module constants of the module
     */
    public SwerveModule(int moduleNumber, SPIKESwerveModConstants moduleConstants) {
        m_configs = new CTREConfigs(moduleConstants);

        m_moduleNumber = moduleNumber;
        m_angleOffset  = moduleConstants.angleOffset;

        // Angle encoder config
        m_angleEncoder = new CANcoder(moduleConstants.cancoderID);
        m_angleEncoder.getConfigurator().apply(m_configs.swerveCANcoderConfig);

        // Angle motor config
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID);
        m_angleMotor.getConfigurator().apply(m_configs.swerveAngleFXConfig);
        resetToAbsolute();

        // Drive motor config
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID);
        m_driveMotor.getConfigurator().apply(m_configs.swerveDriveFXConfig);
        m_driveMotor.setPosition(0);

        m_lastAngle = getState().angle;

        // Enable safety for motors
        m_angleMotor.setSafetyEnabled(true);
        m_driveMotor.setSafetyEnabled(true);
    }

    /**
     * Stops the module
     */
    public void stop() {
        m_driveMotor.set(0);
        m_angleMotor.set(0);
    }

    /**
     * Resets the angle of the angle motor module to absolute (read from CANcoder)
     */
    public void resetToAbsolute() {
        // Convert the CANcoder's degrees to motor rotations
        double rotations = getCANcoder().getRotations();
        m_angleMotor.setPosition(rotations);
        m_targetRotations = rotations;
    }

    /**
     * Sets the desired state of the module
     * @param desiredState The desired state of the module
     * @param isOpenLoop Whether or not to use open loop control (closed loop PID if false)
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // if (desiredState.speedMetersPerSecond < 0.2) {
        //     stop();
        //     return;
        // }

        // desiredState = CTREModuleState.optimize(desiredState, Rotation2d.fromRotations(getState().angle.getRotations() + );
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the neutral mode of the drive and angle motors
     * @param neutralMode The neutral mode to set the motors to
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
        m_driveMotor.setNeutralMode(neutralMode);
    }

    /**
     * Sets the speed of the drive motor
     * @param desiredState The desired state of the module
     * @param isOpenLoop Whether or not to use open loop control (closed loop PID if false)
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.Swerve.maxSpeed;
            m_driveMotor.set(percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio);
            m_driveMotor.setControl(m_velocitySetter.withVelocity(velocity));
        }
    }

    /**
     * Sets the angle of the angle motor
     * @param desiredState The desired state of the module
     */
    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.Swerve.maxSpeed * 0.01)) ? m_lastAngle : desiredState.angle; // Prevent rotating module if speed is less than 1%. Prevents jittering
        // Rotation2d angle = desiredState.angle;

        m_targetRotations = angle.getRotations();

        m_angleMotor.setControl(m_angleSetter.withPosition(m_targetRotations));
        m_lastAngle = angle;
    }

    /**
     * Gets the angle of the module
     * @return The angle of the module
     */
    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValue() + m_angleOffset.getRotations() % 1.0);
    }

    /**
     * Gets the state of the module
     * @return The state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconRPSToMPS(m_driveMotor.getVelocity().getValue(), SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio),
            getAngle()
        );
    }

    /**
     * Gets the angle of the module
     * @return The angle of the module (in rotations)
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_angleMotor.getPosition().getValue());
    }

    /**
     * Gets the position of the module
     * @return The position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driveMotor.getPosition().getValue(), SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio),
            getAngle()
        );
    }
}
