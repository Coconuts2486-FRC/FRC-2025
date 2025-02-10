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

package frc.robot.subsystems.coral_mech;

import frc.robot.util.RBSISubsystem;

public class CoralScorer extends RBSISubsystem {
  private final CoralScorerIO io;

  public CoralScorer(CoralScorerIO io) {
    this.io = io;
  }

  public void runVolts(double volts) {
    io.setVolts(volts);
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void automaticIntake() {
    if (io.getLightStop()) {
      System.out.println("should work");
      io.setPercentOut(1);
    } else {
      System.out.println("angry noises");
      io.setPercentOut(0);
    }
  }

  public void stop() {
    io.stop();
  }

  public boolean getLightStop() {
    return io.getLightStop();
  }

  @Override
  public int[] getPowerPorts() {
    return io.powerPorts;
  }
}
