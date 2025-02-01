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

package frc.robot.subsystems.algae_hands;

import static frc.robot.util.SparkUtil.*;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class AlgaeHandsIOSpark implements AlgaeHandsIO {

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {}

  @Override
  public void stop() {}

  /**
   * Configure the closed-loop PID
   *
   * <p>TODO: This functionality is no longer supported by the REVLib SparkClosedLoopController
   * class. In order to keep control of the AlgaeHands's underlying funtionality, shift everything
   * to SmartMotion control.
   */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    // pid.setP(kP, 0);
    // pid.setI(kI, 0);
    // pid.setD(kD, 0);
    // pid.setFF(0, 0);
  }
}
