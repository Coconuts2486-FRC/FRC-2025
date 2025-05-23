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

package frc.robot.subsystems.algae_mech;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeMechIO {

  public final int[] powerPorts = {};

  @AutoLog
  public static class AlgaeMechIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(AlgaeMechIOInputs inputs) {}

  public default void stop() {}

  public default void setCoast() {}

  public default void setBrake() {}

  public default void pivotToPosition(double position) {}

  /** Updates the set of loggable inputs. */

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setPercent(double percent) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}

  public default double getCurrent() {
    return 0.0;
  }

  public default double getEncoderPose() {
    return .209;
  }
}
