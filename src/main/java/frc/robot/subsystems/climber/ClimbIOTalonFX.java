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

package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimbConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;

/** Algae Mechanism TalonFX-controlled Hardware Class */
public class ClimbIOTalonFX implements ClimbIO {

  // Define the hardware
  private final Servo climbExtender = new Servo(CANandPowerPorts.CLIMB_SERVO);
  private final TalonFX m_climbMotor =
      new TalonFX(CANandPowerPorts.CLIMB.getDeviceNumber(), CANandPowerPorts.CLIMB.getBus());
  // TODO: Convert this to run on a CANdi; fuse to CLIMB motor
  private final DutyCycleEncoder m_climbEncoder =
      new DutyCycleEncoder(CANandPowerPorts.CLIMB_PIVOT_ENCODER);

  // CTRE control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Status signals from CTRE for logging
  private final StatusSignal<Angle> climbPosition = m_climbMotor.getPosition();
  private final StatusSignal<AngularVelocity> climbVelocity = m_climbMotor.getVelocity();
  private final StatusSignal<Voltage> climbAppliedVolts = m_climbMotor.getMotorVoltage();
  private final StatusSignal<Current> climbCurrent = m_climbMotor.getSupplyCurrent();

  // Power ports
  public final int[] powerPorts = {CANandPowerPorts.CLIMB.getPowerPort()};

  // Define the configuration object
  private TalonFXConfiguration climbConfig = new TalonFXConfiguration();

  private final PIDController pid = new PIDController(kPReal, 0, kDReal);

  /** Constructor for using a TalonFX to drive the CLIMB mechanism */
  public ClimbIOTalonFX() {

    // Set and apply TalonFX Configurations
    climbConfig.Slot0 =
        new Slot0Configs().withKP(kPReal).withKI(0.0).withKD(kDReal).withKS(kSReal).withKV(kVReal);
    climbConfig.TorqueCurrent.PeakForwardTorqueCurrent = PowerConstants.kMotorPortMaxCurrent;
    climbConfig.TorqueCurrent.PeakReverseTorqueCurrent = -PowerConstants.kMotorPortMaxCurrent;
    climbConfig.CurrentLimits.StatorCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Constants.loopPeriodSecs;
    climbConfig.MotorOutput.NeutralMode =
        switch (kClimbIdle) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    PhoenixUtil.tryUntilOk(5, () -> m_climbMotor.getConfigurator().apply(climbConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, climbPosition, climbVelocity, climbAppliedVolts, climbCurrent);
    m_climbMotor.optimizeBusUtilization();
  }

  /** Update the inputs for / from logs */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(climbPosition, climbVelocity, climbAppliedVolts, climbCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(climbPosition.getValueAsDouble()) / kClimbGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(climbVelocity.getValueAsDouble()) / kClimbGearRatio;
    inputs.appliedVolts = climbAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {climbCurrent.getValueAsDouble()};
  }

  /** Configure the PID */
  @Override
  public void configPID(double kP, double kI, double kD) {
    Slot0Configs pid = new Slot0Configs();
    pid.withKP(kP);
    pid.withKI(kI);
    pid.withKD(kD);
    m_climbMotor.getConfigurator().apply(pid);
  }

  /**
   * Run the CLIMB motor at a specified voltage
   *
   * <p>NOTE: This is only used under MANUAL CONTROL for development and debugging, and should not
   * be used in competition!!
   */
  @Override
  public void setVoltage(double volts) {
    m_climbMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  /** Run the CLIMB motor at a specified duty cycle */
  @Override
  public void setPercent(double percent) {
    m_climbMotor.setControl(new DutyCycleOut(percent).withEnableFOC(true));
  }

  /** Set the ratcheting servo position */
  @Override
  public void turnClimbServo(double position) {
    climbExtender.set(position);
  }

  /** Command the CLIMB motor to a set position */
  @Override
  public void twistMotorToPosition(double position) {

    m_climbMotor.setControl(
        new DutyCycleOut(-pid.calculate(m_climbEncoder.get(), position)).withEnableFOC(true));
  }

  /** Get the CLIMB encoder position */
  @Override
  public double getEncoderPose() {
    return m_climbEncoder.get();
  }
}
