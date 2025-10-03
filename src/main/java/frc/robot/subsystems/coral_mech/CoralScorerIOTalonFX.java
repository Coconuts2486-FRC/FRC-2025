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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;

public class CoralScorerIOTalonFX implements CoralScorerIO {

  // Define hardware
  private final TalonFX m_coralMotor = new TalonFX(CANandPowerPorts.CORAL_MECH.getDeviceNumber());
  // TODO: Convert this to run on a CANdi
  private final DigitalInput m_lightStop = new DigitalInput(CANandPowerPorts.CORAL_LIGHT_STOP);

  // CTRE control requests
  // private final VoltageOut voltageRequest = new VoltageOut(0);
  // private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  // private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  // private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  // private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
  //     new PositionTorqueCurrentFOC(0.0);
  // private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
  //     new VelocityTorqueCurrentFOC(0.0);

  // Status signals from CTRE for logging
  private final StatusSignal<Angle> coralPosition = m_coralMotor.getPosition();
  private final StatusSignal<AngularVelocity> coralVelocity = m_coralMotor.getVelocity();
  private final StatusSignal<Voltage> coralAppliedVolts = m_coralMotor.getMotorVoltage();
  private final StatusSignal<Current> coralCurrent = m_coralMotor.getSupplyCurrent();

  // Power ports
  public final int[] powerPorts = {CANandPowerPorts.CORAL_MECH.getPowerPort()};

  // Define the configuration object
  private TalonFXConfiguration coralConfig = new TalonFXConfiguration();

  /** Constructor for using a TalonFX to drive the CORAL mechanism */
  public CoralScorerIOTalonFX() {

    // Set and apply TalonFX Configurations
    coralConfig.Slot0 = new Slot0Configs().withKP(kPReal).withKI(0.0).withKD(kDReal);
    coralConfig.TorqueCurrent.PeakForwardTorqueCurrent = PowerConstants.kMotorPortMaxCurrent;
    coralConfig.TorqueCurrent.PeakReverseTorqueCurrent = -PowerConstants.kMotorPortMaxCurrent;
    coralConfig.CurrentLimits.StatorCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    coralConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    coralConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    coralConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    coralConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Constants.loopPeriodSecs;
    coralConfig.MotorOutput.NeutralMode =
        switch (kCoralIdle) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    PhoenixUtil.tryUntilOk(5, () -> m_coralMotor.getConfigurator().apply(coralConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, coralPosition, coralVelocity, coralAppliedVolts, coralCurrent);
    m_coralMotor.optimizeBusUtilization();
  }

  /** Update the inputs for / from logs */
  @Override
  public void updateInputs(CoralScorerIOInputs inputs) {
    BaseStatusSignal.refreshAll(coralPosition, coralVelocity, coralAppliedVolts, coralCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(coralPosition.getValueAsDouble()) / kCoralGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(coralVelocity.getValueAsDouble()) / kCoralGearRatio;
    inputs.appliedVolts = coralAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {coralCurrent.getValueAsDouble()};
    inputs.lightStop = m_lightStop.get();
  }

  /** Run the CORAL mechanism at a specified velocity */
  @Override
  public void setVelocity(double velocity) {
    m_coralMotor.setControl(new VelocityDutyCycle(velocity).withEnableFOC(true));
  }

  /** Run the CORAL mechanism at a specified percent duty cycle */
  @Override
  public void setPercent(double percentOut) {
    m_coralMotor.setControl(new DutyCycleOut(percentOut).withEnableFOC(true));
  }

  /** Get the status of the light stop */
  @Override
  public boolean getLightStop() {
    return m_lightStop.get();
  }

  /** Get the motor output percent */
  @Override
  public double getPercent() {
    return m_coralMotor.getDutyCycle().getValueAsDouble();
  }
}
