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

package frc.robot.subsystems.LED;

import frc.robot.util.RBSISubsystem;

public class LED extends RBSISubsystem {
  private LEDIO io;

  public LED(LEDIO io) {
    this.io = io;
  }

  public void rainbowTwinkle() {
    io.rainbowTwinkle();
  }

  public void off() {
    io.off();
  }

  public void scoreReady(boolean lightstop) {
    if (lightstop) {
      io.scoreReady();
    } else {
      io.rainbowTwinkle();
    }
  }
}
