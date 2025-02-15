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

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  public final int[] powerPorts = {};

  @AutoLog
  public static class ClimbIOInputs {}

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void turnClimbServo(double position) {}

  public default void twistMotorToPosition(double positionInRotations) {}

  public default void configPID(double kP, double kI, double kD) {}

  public default void twistMotorVolts(double volts) {}

  public default void setMotorPercent (double percent) {}

  public default double getEncoderPose () {
    return 0.0;
  }

  public default void setCoast() {}

  public default void setBrake() {}
}
