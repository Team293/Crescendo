package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.LauncherIOTalonFX;

public class LauncherCommand extends Command {
  private final LauncherIOTalonFX launcher;
  private double targetSpeed = 10d; // todo

  public LauncherCommand(LauncherIOTalonFX launcher) {
    this.launcher = launcher;
  }

  @Override
  public void execute() {
    launcher.setSpeed(targetSpeed);
  }

  public void startLauncher(int seconds) {
    execute();
    try {
      wait(seconds * 100);
    } catch (Exception e) {
      e.printStackTrace();
    }
    end(true);
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setSpeed(0);
  }
}
