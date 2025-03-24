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

package frc.robot.subsystems.intake;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends RBSIIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double pivotEncoder = 0.0;
    public double rollerVelRadPerSec = 0.0;
    public double rollerVolts = 0.0;
    public double[] rollerAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPivotPosition(double position) {}

  public default void rollerDutyCycle(double speed) {}

  public default void setPivotVolts(double volts) {}

  public default void configure(double kP, double kI, double kD) {}

  public default void setRollerVolts(double volts) {}

  public default double getEncoderValue() {
    return 0.0;
  }
}
