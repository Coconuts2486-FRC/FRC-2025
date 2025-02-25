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

import static frc.robot.Constants.AlgaeMechConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;

/** ALGAE mechanism hardware class for TalonFX */
public class AlgaeMechIOTalonFX implements AlgaeMechIO {
  // Define Hardware and controllers
  private final TalonFX m_roller = new TalonFX(CANandPowerPorts.ALGAE_ROLLER.getDeviceNumber());
  private final TalonFX m_pivot = new TalonFX(CANandPowerPorts.ALGAE_PIVOT.getDeviceNumber());
  private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private final DutyCycleEncoder pivotEncoder =
      new DutyCycleEncoder(CANandPowerPorts.ALGAE_PIVOT_ENCODER);

  // Status signals from CTRE for logging
  private final StatusSignal<Angle> algaeRollerPosition = m_roller.getPosition();
  private final StatusSignal<AngularVelocity> algaeRollerVelocity = m_roller.getVelocity();
  private final StatusSignal<Voltage> algaeRollerAppliedVolts = m_roller.getMotorVoltage();
  private final StatusSignal<Current> algaeRollerCurrent = m_roller.getSupplyCurrent();
  private final StatusSignal<Angle> algaePivotPosition = m_pivot.getPosition();
  private final StatusSignal<AngularVelocity> algaePivotVelocity = m_pivot.getVelocity();
  private final StatusSignal<Voltage> algaePivotAppliedVolts = m_pivot.getMotorVoltage();
  private final StatusSignal<Current> algaePivotCurrent = m_pivot.getSupplyCurrent();

  // Power port(s)
  private final int[] powerPorts = {
    CANandPowerPorts.ALGAE_PIVOT.getPowerPort(), CANandPowerPorts.ALGAE_ROLLER.getPowerPort()
  };

  // Set up the Motion Magic instance
  // private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

  /** Constructor */
  public AlgaeMechIOTalonFX() {

    // Roller Motor Configs
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.Slot0 = new Slot0Configs().withKP(kPRoller).withKI(kIRoller).withKD(kDRoller);
    rollerConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0.25);
    rollerConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    PhoenixUtil.tryUntilOk(5, () -> m_roller.getConfigurator().apply(rollerConfig, 0.25));

    // Pivot Motor Configs
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0 = new Slot0Configs().withKP(kPPivot).withKI(kIPivot).withKD(kDPivot);
    pivotConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // // TODO: When we move the REV encoder onto the CANdi, these will be used to fuse the encoder
    // onto the motor controller
    // pivotConfig.Feedback.FeedbackRemoteSensorID = CANDI_PWM;
    // // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
    // RemoteCANcoder
    // pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;
    // pivotConfig.Feedback.RotorToSensorRatio = kAlgaePivotGearRatio;
    PhoenixUtil.tryUntilOk(5, () -> m_pivot.getConfigurator().apply(pivotConfig, 0.25));
  }

  /** Update the inputs for / from logs */
  @Override
  public void updateInputs(AlgaeMechIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        algaeRollerPosition,
        algaeRollerVelocity,
        algaeRollerAppliedVolts,
        algaeRollerCurrent,
        algaePivotPosition,
        algaePivotVelocity,
        algaePivotAppliedVolts,
        algaePivotCurrent);
    inputs.rollerPositionRad =
        Units.rotationsToRadians(algaeRollerPosition.getValueAsDouble()) / kAlgaeRollerGearRatio;
    inputs.rollerVelocityRadPerSec =
        Units.rotationsToRadians(algaeRollerVelocity.getValueAsDouble()) / kAlgaeRollerGearRatio;
    inputs.rollerAppliedVolts = algaeRollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = new double[] {algaeRollerCurrent.getValueAsDouble()};
    inputs.pivotPositionRad =
        Units.rotationsToRadians(algaePivotPosition.getValueAsDouble()) / kAlgaePivotGearRatio;
    inputs.pivotVelocityRadPerSec =
        Units.rotationsToRadians(algaePivotVelocity.getValueAsDouble()) / kAlgaePivotGearRatio;
    inputs.pivotAppliedVolts = algaePivotAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = new double[] {algaePivotCurrent.getValueAsDouble()};
  }

  /**
   * Set the pivot to a specified voltage
   *
   * <p>NOTE: DOES NOT FUNCTION!!!
   *
   * @param volts The input voltage
   */
  @Override
  public void setPivotVoltage(double volts) {}

  /**
   * Set the rollers to a specified voltage
   *
   * <p>NOTE: DOES NOT FUNCTION!!!
   *
   * @param volts The input voltage
   */
  @Override
  public void setRollerVoltage(double volts) {}

  /**
   * Set the velocity of the roller motors
   *
   * @param velocityRadPerSec The specified velocity of the roller motor
   * @param ffVolts The feedforward voltage needed for the motor controller
   */
  @Override
  public void setRollerVelocity(double velocityRadPerSec, double ffVolts) {
    m_roller.setControl(
        new VelocityDutyCycle(Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)));
  }

  /**
   * Set the roller motor to a duty cycle percentage
   *
   * @param percent The motor duty cycle percentage
   */
  @Override
  public void setRollerPercent(double percent) {
    m_roller.setControl(new DutyCycleOut(percent));
  }

  /** Stop the motors */
  @Override
  public void stop() {
    m_roller.stopMotor();
    m_pivot.stopMotor();
  }

  /**
   * Move the pivot to a specified position
   *
   * @param position The position to which to move the pivot
   */
  @Override
  public void pivotToPosition(double position) {
    try (PIDController pivotPID = new PIDController(kPPivot, kIPivot, kDPivot)) {
      m_pivot.setControl(new DutyCycleOut(-pivotPID.calculate(pivotEncoder.get(), position)));
      // m_pivot.setControl(m_motionMagic.withPosition(position).withEnableFOC(true));
    }
  }

  /** Get the current consumed by the roller motor */
  @Override
  public double getRollerCurrent() {
    return m_roller.getSupplyCurrent().getValueAsDouble();
  }

  /** Get the pivot encoder position */
  @Override
  public double getPivotEncoderPose() {
    return pivotEncoder.get();
  }

  /** Return the list of PDH power ports used for this mechanism */
  @Override
  public int[] getPowerPorts() {
    return powerPorts;
  }
}
