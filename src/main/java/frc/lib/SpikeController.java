package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SpikeController extends CommandXboxController {
  private double deadband = 0.0;
  // Bypass the deadband in this controller if needed
  // by calling controller.raw.getLeftX() etc.
  public final CommandXboxController raw;

  public SpikeController(int port) {
    super(port);
    raw = new CommandXboxController(port);
  }

  public SpikeController(int port, double deadband) {
    super(port);
    this.deadband = deadband;
    raw = new CommandXboxController(port);
  }

  private double deadband(double input) {
    return MathUtil.applyDeadband(input, deadband);
  }

  @Override
  public double getLeftX() {
    return deadband(super.getLeftX());
  }

  @Override
  public double getLeftY() {
    return deadband(super.getLeftY());
  }

  @Override
  public double getRightX() {
    return deadband(super.getRightX());
  }

  @Override
  public double getRightY() {
    return deadband(super.getRightY());
  }

  @Override
  public double getLeftTriggerAxis() {
    return deadband(super.getLeftTriggerAxis());
  }

  @Override
  public double getRightTriggerAxis() {
    return deadband(super.getRightTriggerAxis());
  }
}
