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

package frc.robot.subsystems.algae_mech;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.AlgaeMechConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;

/** Algae Mechanism TalonFX-controlled Hardware Class */
public class AlgaeMechIOTalonFX implements AlgaeMechIO {

  // Define the hardware
  private final TalonFX m_rollerMotor =
      new TalonFX(CANandPowerPorts.ALGAE_ROLLER.getDeviceNumber());
  private final TalonFX m_pivotMotor = new TalonFX(CANandPowerPorts.ALGAE_PIVOT.getDeviceNumber());
  // TODO: Convert this to run on a CANdi; fuse to pivot motor
  private final DutyCycleEncoder m_pivotEncoder =
      new DutyCycleEncoder(CANandPowerPorts.ALGAE_PIVOT_ENCODER);
  private final ClosedLoopOutputType m_closedLoopOutput =
      switch (Constants.getPhoenixPro()) {
        case LICENSED -> ClosedLoopOutputType.TorqueCurrentFOC;
        case UNLICENSED -> ClosedLoopOutputType.Voltage;
      };

  // CTRE control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  // private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  // private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  // private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
  //    new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Status signals from CTRE for logging
  private final StatusSignal<Angle> pivotPosition = m_pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = m_pivotMotor.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = m_pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> pivotCurrent = m_pivotMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> rollerVelocity = m_rollerMotor.getVelocity();
  private final StatusSignal<Voltage> rollerAppliedVolts = m_rollerMotor.getMotorVoltage();
  private final StatusSignal<Current> rollerCurrent = m_rollerMotor.getSupplyCurrent();

  // Power ports
  public final int[] powerPorts = {
    CANandPowerPorts.ALGAE_PIVOT.getPowerPort(), CANandPowerPorts.ALGAE_ROLLER.getPowerPort()
  };

  // Define the configuration objects
  private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

  private final PIDController pivotController = new PIDController(kPRealPivot, 0, kDRealPivot);

  /** Constructor for using a TalonFX to drive the ALGAE mechanism */
  public AlgaeMechIOTalonFX() {

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
        switch (kAlgaeRollerIdle) {
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
        switch (kAlgaeRollerIdle) {
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

  /** Update the inputs for / from logs */
  @Override
  public void updateInputs(AlgaeMechIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotCurrent,
        rollerVelocity,
        rollerAppliedVolts,
        rollerCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(pivotPosition.getValueAsDouble()) / kAlgaePivotGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(pivotVelocity.getValueAsDouble()) / kAlgaePivotGearRatio;
    inputs.appliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {pivotCurrent.getValueAsDouble()};
    inputs.pivotEncoder = m_pivotEncoder.get();
    inputs.rollerVelRadPerSec =
        Units.rotationsToRadians(rollerVelocity.getValueAsDouble()) / kAlgaeRollerGearRatio;
    inputs.rollerVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerAmps = new double[] {rollerCurrent.getValueAsDouble()};
  }

  /**
   * Configure the PID of... the device?
   *
   * @param kP The proportional gain of the controller
   * @param kI The integral error gain of the controller
   * @param kD The derivative gain of the controller
   */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    // leader.getConfigurator().apply(config);
  }

  /**
   * Set the open-loop voltage at which to run the roller motor
   *
   * @param volts The voltage at which to run the roller motor
   */
  @Override
  public void setVoltage(double volts) {
    m_rollerMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  /**
   * Set the open-loop duty cycle percentage at which to run the roller motor
   *
   * @param percent The percent duty cycle at which to run the roller motor
   */
  @Override
  public void setPercent(double percent) {
    m_rollerMotor.setControl(new DutyCycleOut(percent).withEnableFOC(true));
  }

  /**
   * Set the closed-loop velocity at which to run the roller motor
   *
   * @param velocityRadPerSec The velocity at which to run the roller motor, in rad/s
   * @param ffVolts The feedforward voltage to apply to the motor
   */
  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    m_rollerMotor.setControl(
        switch (m_closedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC ->
              velocityTorqueCurrentRequest.withVelocity(RotationsPerSecond.of(velocityRotPerSec));
        });
  }

  /** Stop all motion of the mechanism */
  @Override
  public void stop() {
    m_rollerMotor.stopMotor();
    m_pivotMotor.stopMotor();
  }

  /**
   * Set the pivot to move to a particular position
   *
   * @param position The position to which to move the ALGAE pivot
   */
  @Override
  public void pivotToPosition(double position) {
    m_pivotMotor.setControl(
        new DutyCycleOut(-pivotController.calculate(m_pivotEncoder.get(), position))
            .withEnableFOC(true));
  }

  /**
   * Return the current being drawn by the roller motor
   *
   * <p>This method is designed to be used to stop the roller motor once the ALGAE has been intaken
   * and stalls the motor.
   */
  @Override
  public double getCurrent() {
    return m_rollerMotor.getSupplyCurrent().getValueAsDouble();
  }

  /** Get the current reading of the pivot absolute encoder */
  @Override
  public double getEncoderPose() {
    return m_pivotEncoder.get();
  }
}
