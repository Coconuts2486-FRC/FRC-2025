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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;

/** CORAL scorer hardware class for TalonFX */
public class CoralScorerIOTalonFX implements CoralScorerIO {

  private final DigitalInput m_lightStop = new DigitalInput(CANandPowerPorts.CORAL_LIGHT_STOP);
  private final TalonFX m_coralIndexer = new TalonFX(CANandPowerPorts.CORAL_MECH.getDeviceNumber());
  private final int[] powerPorts = {CANandPowerPorts.CORAL_MECH.getPowerPort()};
  private final TalonFXConfiguration coralConfig = new TalonFXConfiguration();

  // Status signals from CTRE for logging
  private final StatusSignal<Angle> coralPosition = m_coralIndexer.getPosition();
  private final StatusSignal<AngularVelocity> coralVelocity = m_coralIndexer.getVelocity();
  private final StatusSignal<Voltage> coralAppliedVolts = m_coralIndexer.getMotorVoltage();
  private final StatusSignal<Current> coralCurrent = m_coralIndexer.getSupplyCurrent();

  /** Constructor */
  public CoralScorerIOTalonFX() {
    // Set motor configurations
    coralConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    coralConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Coral Mech should always have neutral mode COAST
    coralConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    coralConfig.Slot0 = new Slot0Configs().withKP(kPReal).withKI(kIReal).withKD(kDReal);
    PhoenixUtil.tryUntilOk(5, () -> m_coralIndexer.getConfigurator().apply(coralConfig, 0.25));
  }

  // /** Update the inputs for / from logs */
  // @Override
  // public void updateInputs(CoralScorerIOInputs inputs) {
  //   BaseStatusSignal.refreshAll(coralPosition, coralVelocity, coralAppliedVolts, coralCurrent);
  //   inputs.positionRad =
  //       Units.rotationsToRadians(coralPosition.getValueAsDouble()) / kCoralGearRatio;
  //   inputs.velocityRadPerSec =
  //       Units.rotationsToRadians(coralVelocity.getValueAsDouble()) / kCoralGearRatio;
  //   inputs.appliedVolts = coralAppliedVolts.getValueAsDouble();
  //   inputs.currentAmps = new double[] {coralCurrent.getValueAsDouble()};
  // }

  /** Return the value of the digital input from the light stop */
  @Override
  public boolean getLightStop() {
    return m_lightStop.get();
  }

  /**
   * Run the motor at the provided voltage
   *
   * <p>This method is used for development and calibration only, SHOULD NOT BE USED IN COMPETITION
   *
   * @param volts The voltage at which to run the motor
   */
  @Override
  public void setVoltage(double volts) {
    m_coralIndexer.setControl(new VoltageOut(volts));
  }

  /**
   * Run the motor at the provided duty cycle percentage
   *
   * @param percentOut The duty cycle at which to run the motor
   */
  public void setPercentOut(double percentOut) {
    m_coralIndexer.setControl(new DutyCycleOut(percentOut));
  }

  /**
   * Run the motor at the provided velocity
   *
   * @param velocity The velocity (in rotations per second) at which to run the motor
   */
  @Override
  public void setVelocity(double velocity) {
    m_coralIndexer.setControl(new VelocityDutyCycle(velocity));
  }

  /** Get the motor output percent */
  @Override
  public double getPercent() {
    return m_coralIndexer.getDutyCycle().getValueAsDouble();
  }

  /** Return the list of PDH power ports used for this mechanism */
  @Override
  public int[] getPowerPorts() {
    return powerPorts;
  }
}
