package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;

public class LEDCommand extends Command {
  LED led;

  public LEDCommand(LED led) {
    this.led = led;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    led.m_led();
  }

  @Override
  public void end(boolean interrupted) {}
}
