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

import static frc.robot.Constants.CoralMechConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.util.PhoenixUtil;

public class CoralScorerIOTalonFX implements CoralScorerIO {

  private final DigitalInput m_lightStop = new DigitalInput(CANandPowerPorts.CORAL_LIGHT_STOP);
  private final TalonFX m_coralIndexer = new TalonFX(CANandPowerPorts.CORAL_MECH.getDeviceNumber());
  public final int[] powerPorts = {CANandPowerPorts.CORAL_MECH.getPowerPort()};

  public CoralScorerIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Coral Mech should always have neutral mode COAST
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(5, () -> m_coralIndexer.getConfigurator().apply(config, 0.25));

    Slot0Configs pid = new Slot0Configs();
    pid.withKP(kPReal);
    pid.withKI(0.0);
    pid.withKD(kDReal);

    m_coralIndexer.getConfigurator().apply(pid);
    m_coralIndexer.getConfigurator().apply(config);
  }

  @Override
  public boolean getLightStop() {
    return m_lightStop.get();
  }

  /**
   * Run the elevator at the provided voltage
   *
   * <p>This method is used for development and calibration only, SHOULD NOT BE USED IN COMPETITION
   *
   * @param volts The voltage at which to run the elevator
   */
  @Override
  public void setVoltage(double volts) {
    m_coralIndexer.setControl(new VoltageOut(volts));
  }

  public void setPercentOut(double percentOut) {
    m_coralIndexer.setControl(new DutyCycleOut(percentOut));
  }

  @Override
  public void setVelocity(double velocity) {
    m_coralIndexer.setControl(new VelocityDutyCycle(velocity));
  }

  /** Get the motor output percent */
  @Override
  public double getPercent() {
    return m_coralIndexer.getDutyCycle().getValueAsDouble();
  }
}
