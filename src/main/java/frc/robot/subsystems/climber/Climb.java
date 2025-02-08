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

package frc.robot.subsystems.climber;

import frc.robot.util.RBSISubsystem;

public class Climb extends RBSISubsystem {
  private ClimbIO io;
  private boolean rachetToggle;

  public Climb(ClimbIO io) {
    this.io = io;
    io.configPID(1, 0, 0);
    rachetToggle = false;
  }

  public void twistToPosition(double position) {
    io.twistMotorToPosition(position);
  }

  public void manualControl(double controlVolts) {
    io.twistMotorVolts(controlVolts * 3);
  }

  public void rachetToggle(boolean toggleSwitch) {
    if (rachetToggle) {
      io.turnClimbServo(0);
    } else {
      io.turnClimbServo(1);
    }
  }
}
