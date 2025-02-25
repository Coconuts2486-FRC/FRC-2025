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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intakeRoller =
      new TalonFX(CANandPowerPorts.INTAKE_ROLLER.getDeviceNumber());
  private final TalonFX intakePivot = new TalonFX(CANandPowerPorts.INTAKE_PIVOT.getDeviceNumber());
  private final CANcoder encoderActual =
      new CANcoder(CANandPowerPorts.INTAKE_ENCODER.getDeviceNumber());
  private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

  PIDController pivotPID = new PIDController(1.5, 0, 0);

  public final int[] powerPorts = {
    CANandPowerPorts.INTAKE_PIVOT.getPowerPort(), CANandPowerPorts.INTAKE_ROLLER.getPowerPort()
  };

  private final StatusSignal<Angle> pivotPosition = encoderActual.getAbsolutePosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = intakePivot.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = intakePivot.getMotorVoltage();
  private final StatusSignal<Current> pivotCurrent = intakePivot.getSupplyCurrent();

  /** Constructor */
  public IntakeIOTalonFX() {
    // Set and apply TalonFX Configurations
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Feedback.FeedbackRemoteSensorID = CANandPowerPorts.INTAKE_ENCODER.getDeviceNumber();
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = PowerConstants.kMotorPortMaxCurrent;
    pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -PowerConstants.kMotorPortMaxCurrent;
    pivotConfig.CurrentLimits.StatorCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Constants.loopPeriodSecs;
    PhoenixUtil.tryUntilOk(5, () -> intakePivot.getConfigurator().apply(pivotConfig, 0.25));
  }

  /**
   * Update the inputs from the CAN devices
   *
   * @param inputs The inputs
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(pivotPosition.getValueAsDouble()); // 21.42857; // gear ratio
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(pivotVelocity.getValueAsDouble()); // 21.42857; // gear ratio
    inputs.appliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {pivotCurrent.getValueAsDouble()};
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
    intakePivot.setVoltage(volts);
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
    intakeRoller.setVoltage(volts);
  }

  /** Stop the intake pivot and rollers */
  @Override
  public void stop() {
    intakeRoller.stopMotor();
    intakePivot.stopMotor();
  }

  /**
   * Set the pivot to a specified position
   *
   * @param position The angle of the absolute encoder to which to set the pivot
   */
  @Override
  public void setPivotPosition(double position) {
    intakePivot.set(-pivotPID.calculate(pivotPosition.getValueAsDouble(), position));
  }

  /**
   * Set the intake rollers to a specified duty cycle
   *
   * @param dutyCycle The "speed" of the motor in the range from -1.0 to 1.0
   */
  @Override
  public void rollerDutyCycle(double dutyCycle) {
    intakeRoller.set(dutyCycle);
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
    pivotPID.setP(kP);
    pivotPID.setI(kI);
    pivotPID.setD(kD);
  }

  /** Return the current encoder value for the intake pivot */
  @Override
  public double getEncoderValue() {
    return pivotPosition.getValueAsDouble();
  }

  /** Return the list of PDH power ports used for this mechanism */
  @Override
  public int[] getPowerPorts() {
    return powerPorts;
  }
}
