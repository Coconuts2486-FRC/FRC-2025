// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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
package frc.robot.subsystems.CoralScorer;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface CoralScorerIO {

  @AutoLog
  public static class IntakeIOInputs {}

  public default void lightStop(BooleanSupplier lightStop) {}

  public default void setVolts(double volts) {}

  public default boolean getLightStop() {
    return false;
  }

  public default void stop() {}
}
