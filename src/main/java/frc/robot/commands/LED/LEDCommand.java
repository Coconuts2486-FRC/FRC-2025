package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import java.util.function.BooleanSupplier;

public class LEDCommand extends Command {
  LED led;
  BooleanSupplier lightstop;

  public LEDCommand(LED led, BooleanSupplier lightStop) {
    this.led = led;
    this.lightstop = lightStop;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    led.scoreReady(lightstop.getAsBoolean());
    // led.rainbowTwinkle();
    // led.larson();
    // led.police();
  }

  @Override
  public void end(boolean interrupted) {
    led.off();
  }
}
