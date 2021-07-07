// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.ButtonMappingConstants.*;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.classes.TargetPosition2D;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
    private static RobotContainer m_robotContainer = new RobotContainer();

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    // The robot's subsystems
    public final Climb m_climb = new Climb();
    public final Targeting m_targeting = new Targeting();
    public final BallPickup m_ballPickup = new BallPickup();
    public final Feeder m_feeder = new Feeder();
    public final Launcher m_launcher = new Launcher();
    public final LauncherController m_launcherController = new LauncherController(m_launcher);
    public final Kinematics m_kinematics = new Kinematics(new Position2D(0.0,0.0,0.0));
    public final Drivetrain m_drivetrain = new Drivetrain(m_kinematics);

    // Joysticks/Controllers
    private final XboxController xboxController = new XboxController(4);
    private final Joystick operatorRightJoy = new Joystick(2);
    private final Joystick operatorLeftJoy = new Joystick(3);
    private final Joystick rightJoy = new Joystick(1);
    private final Joystick leftJoy = new Joystick(0);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  
    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
    */
    private RobotContainer() 
    {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
        // Smartdashboard Subsystems

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
        // Configure the button bindings
        configureButtonBindings();
        // Configure default commands
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

        // Configure autonomous sendable chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        List<TargetPosition2D>threeByThreeBoxPath = new ArrayList<TargetPosition2D>();
        List<TargetPosition2D>bouncePath = new ArrayList<TargetPosition2D>();
        List<TargetPosition2D>slalomPath = new ArrayList<TargetPosition2D>();
        List<TargetPosition2D>barrelPath = new ArrayList<TargetPosition2D>();
        List<TargetPosition2D>testPath = new ArrayList<TargetPosition2D>();
        
        //Slalom 
        slalomPath.add(new TargetPosition2D(3.500000,1.500000,Math.toRadians(0.000000),5.000000));
        slalomPath.add(new TargetPosition2D(10.302632,8.250000,Math.toRadians(26.565051),5.000000));
        slalomPath.add(new TargetPosition2D(20.000000,7.934211,Math.toRadians(-30.000000),5.000000));
        slalomPath.add(new TargetPosition2D(25.342105,2.210526,Math.toRadians(-1.169139),5.000000));
        slalomPath.add(new TargetPosition2D(28.065789,4.973684,Math.toRadians(91.974934),5.000000));
        slalomPath.add(new TargetPosition2D(24.300000,8.131579,Math.toRadians(-139.398705),5.000000));
        slalomPath.add(new TargetPosition2D(19.342105,1.973684,Math.toRadians(-168.111342),5.000000));
        slalomPath.add(new TargetPosition2D(8.800000,3.473684,Math.toRadians(130.000000),5.000000));
        slalomPath.add(new TargetPosition2D(3.236842,7.697368,Math.toRadians(-180.000000),5.000000));

        //3X3 Square 
        threeByThreeBoxPath.add(new TargetPosition2D(0, 0, Math.toRadians(0.0), 1.5d));
        threeByThreeBoxPath.add(new TargetPosition2D(3.0, 0.0, Math.toRadians(0.0), 1.5d));
        threeByThreeBoxPath.add(new TargetPosition2D(3.0, 3.0, Math.toRadians(180.0), 1.5d));
        threeByThreeBoxPath.add(new TargetPosition2D(0.0, 3.0, Math.toRadians(270.0), 1.5d));
        threeByThreeBoxPath.add(new TargetPosition2D(0.0, 0.0, Math.toRadians(0), 1.5d));

        //Bounce path
        bouncePath.add(new TargetPosition2D(2.500000,7.500000,Math.toRadians(0.000000),2.000000));
        bouncePath.add(new TargetPosition2D(7.500000,11.500000,Math.toRadians(90.000000),-2.000000));
        bouncePath.add(new TargetPosition2D(8.500000,7.900000,Math.toRadians(-65.000000),-2.000000));
        bouncePath.add(new TargetPosition2D(10.000000,5.000000,Math.toRadians(-60.000000),-2.000000));
        bouncePath.add(new TargetPosition2D(12.500000,2.500000,Math.toRadians(0.000000),-2.000000));
        bouncePath.add(new TargetPosition2D(15.000000,5.000000,Math.toRadians(90.000000),-2.000000));
        bouncePath.add(new TargetPosition2D(15.000000,11.000000,Math.toRadians(90.000000),2.000000));
        bouncePath.add(new TargetPosition2D(15.000000,5.000000,Math.toRadians(-90.000000),2.000000));
        bouncePath.add(new TargetPosition2D(17.500000,1.500000,Math.toRadians(0.000000),2.000000));
        bouncePath.add(new TargetPosition2D(20.000000,1.500000,Math.toRadians(0.000000),2.000000));
        bouncePath.add(new TargetPosition2D(22.500000,5.000000,Math.toRadians(90.000000),2.000000));
        bouncePath.add(new TargetPosition2D(22.500000,9.500000,Math.toRadians(90.000000),2.000000));
        bouncePath.add(new TargetPosition2D(22.500000,11.000000,Math.toRadians(90.000000),-2.000000));
        bouncePath.add(new TargetPosition2D(25.000000,5.5000000,Math.toRadians(0.000000),-2.500000));
        bouncePath.add(new TargetPosition2D(27.500000,5.0000000,Math.toRadians(0.000000),-2.000000));

        //Barrel Path
        barrelPath.add(new TargetPosition2D(2.500000,7.500000,Math.toRadians(0.000000),5));
        barrelPath.add(new TargetPosition2D(16.000000,5.000000,Math.toRadians(270.000000),5));
        barrelPath.add(new TargetPosition2D(12.908000,2.000000,Math.toRadians(180.000000),5));
        barrelPath.add(new TargetPosition2D(9.500000,5.339000,Math.toRadians(90.000000),5));
        barrelPath.add(new TargetPosition2D(13.314000,8.000000,Math.toRadians(0.000000),5));
        barrelPath.add(new TargetPosition2D(23.500000,10.000000,Math.toRadians(90.000000),5));
        barrelPath.add(new TargetPosition2D(20.500000,13.500000,Math.toRadians(180.000000),5));
        barrelPath.add(new TargetPosition2D(17.300000,10.000000,Math.toRadians(-90.000000),5));
        barrelPath.add(new TargetPosition2D(23.000000,2.500000,Math.toRadians(-5.000000),5));
        barrelPath.add(new TargetPosition2D(27.000000,5.000000,Math.toRadians(90.000000),5));
        barrelPath.add(new TargetPosition2D(25.000000,8.500000,Math.toRadians(180.000000),9));
        barrelPath.add(new TargetPosition2D(2.500000,8.500000,Math.toRadians(195.000000),5));
        
        // barrelPath.add(new TargetPosition2D(2.500000,7.500000,Math.toRadians(0.000000),5.000000));
        // barrelPath.add(new TargetPosition2D(14.000000,7.700000,Math.toRadians(-45.000000),3.000000));
        // barrelPath.add(new TargetPosition2D(16.000000,5.000000,Math.toRadians(270.000000),3.000000));
        // barrelPath.add(new TargetPosition2D(12.908000,2.000000,Math.toRadians(180.000000),3.000000));
        // barrelPath.add(new TargetPosition2D(9.500000,5.339000,Math.toRadians(90.000000),3.000000));
        // barrelPath.add(new TargetPosition2D(13.314000,7.200000,Math.toRadians(2.045000),4.000000));
        // barrelPath.add(new TargetPosition2D(22.500000,10.000000,Math.toRadians(90.000000),3.000000));
        // barrelPath.add(new TargetPosition2D(21.000000,12.500000,Math.toRadians(180.000000),3.000000));
        // barrelPath.add(new TargetPosition2D(18.000000,9.000000,Math.toRadians(-69.193000),4.500000));
        // barrelPath.add(new TargetPosition2D(23.000000,3.000000,Math.toRadians(-10.000000),5.500000));
        // barrelPath.add(new TargetPosition2D(28.000000,4.000000,Math.toRadians(90.000000),4.000000));
        // barrelPath.add(new TargetPosition2D(25.000000,8.000000,Math.toRadians(180.000000),3.000000));
        // barrelPath.add(new TargetPosition2D(2.500000,7.500000,Math.toRadians(-180.000000),4.000000));


        testPath.add(new TargetPosition2D(0, 0, Math.toRadians(0),1.0d));
        testPath.add(new TargetPosition2D(5, 5, Math.toRadians(90),-1.0d));
        testPath.add(new TargetPosition2D(0, 0, Math.toRadians(180),1.0d));
        //straightPath.add(new TargetPosition2D(10, 5, Math.toRadians(90),1.0d));



        m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand(m_drivetrain, m_kinematics, threeByThreeBoxPath));
        m_chooser.addOption("3x3 Autonav", new AutonomousCommand(m_drivetrain, m_kinematics, threeByThreeBoxPath));
        m_chooser.addOption("Slalom Autonav", new AutonomousCommand(m_drivetrain, m_kinematics, slalomPath));
        m_chooser.addOption("Bounce Autonav", new AutonomousCommand(m_drivetrain, m_kinematics, bouncePath));
        m_chooser.addOption("Barrel Autonav", new AutonomousCommand(m_drivetrain, m_kinematics, barrelPath));
        m_chooser.addOption("Test Autonav", new AutonomousCommand(m_drivetrain, m_kinematics, testPath));

        //Setting default command for drivetrain as VelocityDrive
        m_drivetrain.setDefaultCommand(new ArcadeDrive( m_drivetrain, xboxController));

        SmartDashboard.putData("Auto Mode", m_chooser);
    }

    public static RobotContainer getInstance() 
    {
        return m_robotContainer;
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() 
    {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
        // Create some buttons
        final JoystickButton geckoDownBtn = new JoystickButton(operatorRightJoy, GECKO_DOWN_BTN);        
        geckoDownBtn.whenPressed(new GeckoDown( m_ballPickup ) ,true);
        SmartDashboard.putData("geckoDownBtn",new GeckoDown( m_ballPickup ) );

        final JoystickButton geckoUpBtn = new JoystickButton(operatorRightJoy, GECKO_UP_BTN);        
        geckoUpBtn.whenPressed(new GeckoUp( m_ballPickup, m_feeder ) ,true);
        SmartDashboard.putData("geckoUpBtn",new GeckoUp( m_ballPickup, m_feeder ) );

        final JoystickButton retractClimberBtn = new JoystickButton(operatorRightJoy, RETRACT_CLIMBER_BTN);        
        retractClimberBtn.whenPressed(new RetractClimber( m_climb ) ,true);
        SmartDashboard.putData("retractClimberBtn",new RetractClimber( m_climb ) );

        final JoystickButton extendClimberBtn = new JoystickButton(operatorRightJoy, EXTEND_CLIMBER_BTN);        
        extendClimberBtn.whenPressed(new ExtendClimber( m_climb ) ,true);
        SmartDashboard.putData("extendClimberBtn",new ExtendClimber( m_climb ) );

        final JoystickButton toggleGeckoBtn = new JoystickButton(operatorRightJoy, TOGGLE_GECKO_BTN);        
        toggleGeckoBtn.whenPressed(new ToggleGecko( m_ballPickup, m_feeder ) ,true);
        SmartDashboard.putData("toggleGeckoBtn",new ToggleGecko( m_ballPickup, m_feeder ) );

        final JoystickButton locateTargetBtn = new JoystickButton(operatorRightJoy, LOCATE_TARGET_BTN);        
        locateTargetBtn.whileHeld(new LocateTarget( m_drivetrain, m_targeting ) ,true);
        SmartDashboard.putData("locateTargetBtn",new LocateTarget( m_drivetrain, m_targeting ) );

        final JoystickButton fireBtn = new JoystickButton(operatorLeftJoy, FIRE_BTN);        
        fireBtn.whileHeld(new Fire( m_feeder, m_launcher, m_launcherController) ,true);
        SmartDashboard.putData("fireBtn",new Fire( m_feeder, m_launcher, m_launcherController) );

        final JoystickButton locateBtn = new JoystickButton(leftJoy, LOCATE_BTN);        
        locateBtn.whileHeld(new LocateTarget( m_drivetrain, m_targeting ) ,true);
        SmartDashboard.putData("locateBtn",new LocateTarget( m_drivetrain, m_targeting ) );

        final JoystickButton shooterPistonUpBtn = new JoystickButton(operatorLeftJoy, SHOOTER_PISTON_UP_BTN);
        shooterPistonUpBtn.whenPressed(new ShooterPistonUp(m_launcher), true);
        SmartDashboard.putData("shooterPistonUpBtn", new ShooterPistonUp(m_launcher));

        final JoystickButton shooterPistonDownBtn = new JoystickButton(operatorLeftJoy, SHOOTER_PISTON_UP_BTN);
        shooterPistonDownBtn.whenPressed(new ShooterPistonDown(m_launcher), true);
        SmartDashboard.putData("shooterPistonDownBtn", new ShooterPistonDown(m_launcher));

        //Toggle Green Zone RPMs 
        final JoystickButton greenZoneBtn = new JoystickButton(xboxController, XboxController.Button.kX.value);
        greenZoneBtn.whenPressed(new SetGreenZone(m_launcherController));

        //Toggle Yellow Zone RPMs 
        final JoystickButton yellowZoneBtn = new JoystickButton(xboxController, XboxController.Button.kA.value);
        yellowZoneBtn.whenPressed(new SetYellowZone(m_launcherController));
        
        //Toggle Blue Zone RPMs 
        final JoystickButton blueZoneBtn = new JoystickButton(xboxController, XboxController.Button.kB.value);
        blueZoneBtn.whenPressed(new SetBlueZone(m_launcherController));
        
        //Toggle Red Zone RPMs 
        final JoystickButton redZoneBtn = new JoystickButton(xboxController, XboxController.Button.kY.value);
        redZoneBtn.whenPressed(new SetRedZone(m_launcherController));

        final JoystickButton xboxFireBtn = new JoystickButton(xboxController, XboxController.Button.kBumperRight.value);
        
        //Raise the launcher piston
        final POVButton dpadUpButton = new POVButton(xboxController, 0);
        dpadUpButton.whenPressed(new ShooterPistonUp(m_launcher));

        //Lower the launcher piston
        final POVButton dpadDownButton = new POVButton(xboxController, 180);
        dpadDownButton.whenPressed(new ShooterPistonDown(m_launcher));

        SmartDashboard.putData("Reset Kinematics", new ResetKinematics(m_drivetrain));

        xboxFireBtn.whileHeld(new Fire(m_feeder, m_launcher, m_launcherController));

        final JoystickButton xboxTargetBtn = new JoystickButton(xboxController, XboxController.Button.kBumperLeft.value);
        xboxTargetBtn.whileHeld(    new LocateTarget(m_drivetrain, m_targeting));
        
        SmartDashboard.putData("AutoFeedBtn",new AutoFeed(m_drivetrain,m_feeder,m_launcher,m_launcherController,m_targeting));

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getleftJoy() 
    {
        return leftJoy;
    }

    public Joystick getrightJoy() 
    {
        return rightJoy;
    }

    public Joystick getoperatorLeftJoy() 
    {
        return operatorLeftJoy;
    }

    public Joystick getoperatorRightJoy()
    {
        return operatorRightJoy;
    }

    public XboxController getxboxController() 
    {
      return xboxController;
    }
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
        // The selected command will be run in autonomous
        return m_chooser.getSelected();
    }

    public Command getTeleopCommand()
    {
        return new ArcadeDrive(m_drivetrain, xboxController);
    }
}
