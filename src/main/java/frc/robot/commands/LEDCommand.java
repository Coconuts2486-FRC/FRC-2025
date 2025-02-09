// Copyright (c) 2025 FRC 2486
// http://github.com/Coconuts2486-FRC
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

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
