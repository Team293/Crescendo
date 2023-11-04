// RobotBuilder Version: 4.0
// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.classes.SpikeController;
import frc.robot.commands.OrientedDrive;
import frc.robot.commands.SequentialAutoCommand;
import frc.robot.commands.SequentialAutoCommand.StartPositions;
import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
    // Robots Subsystems
    private static RobotContainer m_robotContainer = new RobotContainer();
    public final Drivetrain m_drivetrain = new Drivetrain();

    // Create two new controllers with the SpikeController class, which handles deadband and stick direction automatically
    public static final double DEADBAND = 0.05d;
    public final SpikeController m_driverController = new SpikeController(new XboxController(0), DEADBAND, "Driver");
    public final SpikeController m_operatorController = new SpikeController(new XboxController(0), DEADBAND, "Operator");

    public final SendableChooser<Command> m_driveChooser = new SendableChooser<>();
    public final SendableChooser<StartPositions> m_autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Setting default command for drivetrain as VelocityDrive
        m_drivetrain.setDefaultCommand(new OrientedDrive(m_drivetrain, m_driverController, true));
    }

    public static RobotContainer getInstance() {
        return m_robotContainer;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /******** Operator Controls ********/
        // final JoystickButton xboxCalibrateExtenderBtn = new JoystickButton(m_operatorXboxController,
        //     XboxController.Button.kRightBumper.value);
        // xboxCalibrateExtenderBtn.whileTrue(new CalibrateExtender(m_arm));

        // Added options to the dropdown for driveChooser and putting it into
        // smartdashboard
        m_driveChooser.setDefaultOption("Field Oriented Drive", new OrientedDrive(m_drivetrain, m_driverController, true));
        m_driveChooser.addOption("Robot Oriented Drive", new OrientedDrive(m_drivetrain, m_driverController, false));
        SmartDashboard.putData(m_driveChooser);

        m_autoChooser.setDefaultOption("Don't Move", SequentialAutoCommand.StartPositions.DONT_MOVE);
        m_autoChooser.addOption("Drive Backward", SequentialAutoCommand.StartPositions.DRIVE_BACKWARD);
        m_autoChooser.addOption("Left Side Score", SequentialAutoCommand.StartPositions.LEFT_SIDE_SCORE);
    }

    private Command getDriveCommand() {
        return m_driveChooser.getSelected();
    }

    public void setNeutralMode(NeutralMode nm) {
        m_drivetrain.setNeutralMode(nm);
    }

    public void setDefaultDrive() {
        m_drivetrain.setDefaultCommand(getDriveCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        // When either the alliance colour check or the location check fails it defaults
        // to the blue left side
        Command autoCommand = null;
    
  
        autoCommand = new SequentialAutoCommand(m_drivetrain, m_autoChooser.getSelected());

        return autoCommand;
    }
}
