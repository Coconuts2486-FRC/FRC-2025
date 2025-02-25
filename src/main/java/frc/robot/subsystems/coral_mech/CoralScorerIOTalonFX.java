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

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANandPowerPorts;

public class CoralScorerIOTalonFX implements CoralScorerIO {

  private final DigitalInput lightStop = new DigitalInput(2);
  private final TalonFX coralIndexer = new TalonFX(CANandPowerPorts.CORAL_MECH.getDeviceNumber());
  public final int[] powerPorts = {CANandPowerPorts.CORAL_MECH.getPowerPort()};

  public CoralScorerIOTalonFX() {
    var pid = new Slot0Configs();
    pid.withKP(1);
    pid.withKI(0);
    pid.withKD(0);
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    coralIndexer.getConfigurator().apply(pid);
    coralIndexer.getConfigurator().apply(config);
  }

  @Override
  public boolean getLightStop() {
    return lightStop.get();
  }

  public void setPercentOut(double percentOut) {
    coralIndexer.setControl(new DutyCycleOut(percentOut));
  }

  @Override
  public void setVelocity(double velocity) {
    coralIndexer.setControl(new VelocityDutyCycle(velocity));
  }

  /** Get the motor output percent */
  @Override
  public double getPercent() {
    return coralIndexer.getDutyCycle().getValueAsDouble();
  }
}
