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

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;

/** Intake TalonFX-controlled Hardware Class */
public class IntakeIOTalonFX implements IntakeIO {

  // Define the hardware
  private final TalonFX m_rollerMotor =
      new TalonFX(CANandPowerPorts.INTAKE_ROLLER.getDeviceNumber());
  private final TalonFX m_pivotMotor = new TalonFX(CANandPowerPorts.INTAKE_PIVOT.getDeviceNumber());
  private final CANcoder m_pivotEncoder =
      new CANcoder(CANandPowerPorts.INTAKE_ENCODER.getDeviceNumber());

  // CTRE control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Status signals from CTRE for logging
  private final StatusSignal<Angle> pivotPosition = m_pivotEncoder.getPosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = m_pivotEncoder.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = m_pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> pivotCurrent = m_pivotMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> rollerVelocity = m_rollerMotor.getVelocity();
  private final StatusSignal<Voltage> rollerAppliedVolts = m_rollerMotor.getMotorVoltage();
  private final StatusSignal<Current> rollerCurrent = m_rollerMotor.getSupplyCurrent();

  // Power ports
  public final int[] powerPorts = {
    CANandPowerPorts.INTAKE_PIVOT.getPowerPort(), CANandPowerPorts.INTAKE_ROLLER.getPowerPort()
  };

  // Define the configuration objects
  private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

  private final PIDController pivotController = new PIDController(kPRealPivot, 0, kDRealPivot);

  /** Constructor for using a TalonFX to drive the intake */
  public IntakeIOTalonFX() {

    // Set and apply TalonFX Configurations
    rollerConfig.Slot0 = new Slot0Configs().withKP(kPRealRoller).withKI(0.0).withKD(kDRealRoller);
    rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = PowerConstants.kMotorPortMaxCurrent;
    rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -PowerConstants.kMotorPortMaxCurrent;
    rollerConfig.CurrentLimits.StatorCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Constants.loopPeriodSecs;
    rollerConfig.MotorOutput.NeutralMode =
        switch (kIntakeRollerIdle) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    PhoenixUtil.tryUntilOk(5, () -> m_rollerMotor.getConfigurator().apply(rollerConfig, 0.25));

    pivotConfig.Slot0 = new Slot0Configs().withKP(kPRealPivot).withKI(0.0).withKD(kDRealPivot);
    pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = PowerConstants.kMotorPortMaxCurrent;
    pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -PowerConstants.kMotorPortMaxCurrent;
    pivotConfig.CurrentLimits.StatorCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Constants.loopPeriodSecs;
    pivotConfig.MotorOutput.NeutralMode =
        switch (kIntakeRollerIdle) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    PhoenixUtil.tryUntilOk(5, () -> m_pivotMotor.getConfigurator().apply(pivotConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rollerVelocity, rollerAppliedVolts, rollerCurrent);
    m_rollerMotor.optimizeBusUtilization();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    m_pivotMotor.optimizeBusUtilization();
  }

  /**
   * Update the inputs from the CAN devices
   *
   * @param inputs The inputs
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotCurrent,
        rollerVelocity,
        rollerAppliedVolts,
        rollerCurrent);
    inputs.positionRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble()) / 2.0;
    inputs.velocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble()) / 2.0;
    inputs.appliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {pivotCurrent.getValueAsDouble()};
    inputs.rollerVelRadPerSec =
        Units.rotationsToRadians(rollerVelocity.getValueAsDouble()) / kIntakeRollerGearRatio;
    inputs.rollerVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerAmps = new double[] {rollerCurrent.getValueAsDouble()};
  }

  /**
   * Configure the PID for the intake pivot
   *
   * @param kP The proportional gain for the PID controller
   * @param kI The integram gain for the PID controller
   * @param kD The derivative gain for the PID controller
   */
  @Override
  public void configure(double kP, double kI, double kD) {
    pivotController.setP(kP);
    pivotController.setI(kI);
    pivotController.setD(kD);
  }

  /**
   * Make the pivot move with an inidicated voltage
   *
   * <p>NOTE: This method should only be used during construction, testing, and tuning -- not in
   * actual operation or competition!
   *
   * @param volts The voltage to apply to the drive motor for the intake pivot
   */
  @Override
  public void setPivotVolts(double volts) {
    m_pivotMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  /**
   * Make the rollers move with an inidicated voltage
   *
   * <p>NOTE: This method should only be used during construction, testing, and tuning -- not in
   * actual operation or competition!
   *
   * @param volts The voltage to apply to the drive motor for the intake rollers
   */
  @Override
  public void setRollerVolts(double volts) {
    m_rollerMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  /**
   * Set the pivot to a specified position
   *
   * @param position The angle of the absolute encoder to which to set the pivot
   */
  @Override
  public void setPivotPosition(double position) {
    m_pivotMotor.setControl(
        new DutyCycleOut(-pivotController.calculate(pivotPosition.getValueAsDouble(), position))
            .withEnableFOC(true));
  }

  /**
   * Set the intake rollers to a specified duty cycle
   *
   * @param dutyCycle The "speed" of the motor in the range from -1.0 to 1.0
   */
  @Override
  public void rollerDutyCycle(double dutyCycle) {
    m_rollerMotor.setControl(new DutyCycleOut(dutyCycle).withEnableFOC(true));
  }

  /** Stop the intake pivot and rollers */
  @Override
  public void stop() {
    m_rollerMotor.stopMotor();
    m_pivotMotor.stopMotor();
  }

  /** Return the current encoder value for the intake pivot */
  @Override
  public double getEncoderValue() {
    return pivotPosition.getValueAsDouble();
  }
}
