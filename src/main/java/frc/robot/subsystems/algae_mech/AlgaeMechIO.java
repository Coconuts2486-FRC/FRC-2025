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

  @AutoLog
  public static class AlgaeMechIOInputs {
    public double rollerPositionRad = 0.0;
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double[] rollerCurrentAmps = new double[] {};
    public double pivotPositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double[] pivotCurrentAmps = new double[] {};
  }

  public default void updateInputs(AlgaeMechIOInputs inputs) {}

  public default void stop() {}

  public default void setCoast() {}

  public default void setBrake() {}

  public default void pivotToPosition(double position) {}

  /** Updates the set of loggable inputs. */

  /** Run open loop at the specified voltage. */
  public default void setPivotVoltage(double volts) {}

  /** Run open loop at the specified voltage. */
  public default void setRollerVoltage(double volts) {}

  public default void setRollerPercent(double percent) {}

  /** Run closed loop at the specified velocity. */
  public default void setRollerVelocity(double velocityRadPerSec, double ffVolts) {}

  public default double getRollerCurrent() {
    return 0.0;
  }

  public default double getPivotEncoderPose() {
    return .209;
  }

  public default int[] getPowerPorts() {
    return new int[1];
  }
}
