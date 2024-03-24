package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private DoubleSolenoid climbSolenoid;
  private Compressor compressor;

  public Climb() {
    compressor = new Compressor(PneumaticsModuleType.REVPH);

    climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    compressor.enableAnalog(90, 120);
  }

  @Override
  public void periodic() {
    // .getNumber("Compresser Pressure", )
    // This method will be called once per scheduler run
  }

  // Put methods for controlling this subsystem here. Call these from Commands.
  public void climberDown() {
    climbSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void climberUp() {
    climbSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void stop() {
    climbSolenoid.set(DoubleSolenoid.Value.kOff);
  }
}
