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

import org.littletonrobotics.junction.AutoLog;

public interface CoralScorerIO {

  public final int[] powerPorts = {};

  @AutoLog
  public static class CoralScorerIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(CoralScorerIOInputs inputs) {}

  public default void setVolts(double volts) {}

  public default void stop() {}

  public default boolean getLightStop() {
    return false;
  }

  public default double getPercent() {
    return 0;
  }

  public default void setPercentOut(double percent) {}

  public default void setVelocity(double velocity) {}

  public default void setCoast() {}

  public default void setBrake() {}
}
