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

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double pivotEncoder = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void turnClimbServo(double position) {}

  public default void twistMotorToPosition(double positionInRotations) {}

  public default void configPID(
      double Ks,
      double Kv,
      double Ka,
      double Kp,
      double Ki,
      double Kd,
      AngularVelocity velocity,
      AngularAcceleration acceleration) {}

  public default void twistMotorVolts(double volts) {}

  public default void setVoltage(double volts) {}

  public default void setMotorPercent(double percent) {}

  public default double getEncoderPose() {
    return 0.0;
  }

  public default void setCoast() {}

  public default void setBrake() {}

  public default int[] getPowerPorts() {
    return new int[1];
  }
}
