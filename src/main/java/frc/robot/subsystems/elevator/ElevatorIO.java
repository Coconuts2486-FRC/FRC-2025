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

package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosistion(Distance posistion) {}

  public default void stop() {}

  public default boolean getBottomStop() {
    return false;
  }

  public default double getElevatorHeight() {
    return 0.0;
  }

  public default void configure(
      double Kg,
      double Ks,
      double Kv,
      double Ka,
      double Kp,
      double Ki,
      double Kd,
      LinearVelocity velocity,
      LinearAcceleration aceleration,
      double jerk) {}

  public default BooleanSupplier isAtPosition() {
    return () -> false;
  }

  public default double getMotorPosition() {
    return 0;
  }

  public default double getCommandedPosition() {
    return 0;
  }

  public default void setVoltage(double volts) {}

  public default void limit(BooleanSupplier limitSwitch) {}

  public default void setCoast() {}

  public default void setBrake() {}

  public default int[] getPowerPorts() {
    return new int[1];
  }
}
