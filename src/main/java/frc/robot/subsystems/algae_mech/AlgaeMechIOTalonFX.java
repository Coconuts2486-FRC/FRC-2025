package frc.robot.subsystems.algae_mech;

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

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class AlgaeMechIOTalonFX implements AlgaeMechIO {
  private final TalonFX roller = new TalonFX(28);
  private final TalonFX pivot = new TalonFX(29);
  private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

  public AlgaeMechIOTalonFX() {
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    roller.getConfigurator().apply(rollerConfig);
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivot.getConfigurator().apply(pivotConfig);
  }

  // Define the leader / follower motors from the Ports section of RobotContainer

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    roller.setControl(
        new VelocityDutyCycle(Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)));
  }

  @Override
  public void stop() {
    roller.stopMotor();
  }

  @Override
  public void pivotToPosition(double position) {
    pivot.setControl(new PositionDutyCycle(position));
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    // leader.getConfigurator().apply(config);
  }
}
