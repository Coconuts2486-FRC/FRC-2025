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

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.MiscFuncs;
import frc.robot.util.PhoenixUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {

  // Define the motor and the limit switch at the bottom of the elevator
  private final TalonFX m_elevatorMotor =
      new TalonFX(CANandPowerPorts.ELEVATOR.getDeviceNumber(), CANandPowerPorts.ELEVATOR.getBus());
  // TODO: Move this onto a CANdi
  private final DigitalInput m_elevatorStop =
      new DigitalInput(CANandPowerPorts.ELEVATOR_BOTTOM_LIMIT);

  // Status signals from CTRE for logging
  private final StatusSignal<Angle> elevatorPosition = m_elevatorMotor.getPosition();
  private final StatusSignal<AngularVelocity> elevatorVelocity = m_elevatorMotor.getVelocity();
  private final StatusSignal<Voltage> elevatorAppliedVolts = m_elevatorMotor.getMotorVoltage();
  private final StatusSignal<Current> elevatorCurrent = m_elevatorMotor.getSupplyCurrent();

  // Power port
  public final int[] powerPorts = {CANandPowerPorts.ELEVATOR.getPowerPort()};

  // Define the configuration object
  private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

  // Set up the Motion Magic instance
  private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

  private Angle m_commandedMotorPosition = Rotations.of(0.);

  /** Constructor for using a TalonFX to drive the elevator */
  public ElevatorIOTalonFX() {

    // Set and apply TalonFX Configurations
    elevatorConfig.Slot0 =
        new Slot0Configs()
            .withKP(ElevatorConstants.kPReal)
            .withKI(0.0)
            .withKD(ElevatorConstants.kDReal)
            .withKS(ElevatorConstants.kSReal)
            .withKV(ElevatorConstants.kVReal)
            .withKG(ElevatorConstants.kGReal);
    elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = PowerConstants.kMotorPortMaxCurrent;
    elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -PowerConstants.kMotorPortMaxCurrent;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Constants.loopPeriodSecs;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // NOTE: Elevators should ALWAYS have neutral mode BRAKE
    // switch (kElevatorIdle) {
    //   case COAST -> NeutralModeValue.Coast;
    //   case BRAKE -> NeutralModeValue.Brake;
    // };
    PhoenixUtil.tryUntilOk(5, () -> m_elevatorMotor.getConfigurator().apply(elevatorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, elevatorPosition, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);
    m_elevatorMotor.optimizeBusUtilization();
  }

  /** Update the inputs for / from logs */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        elevatorPosition, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(elevatorPosition.getValueAsDouble()) / kElevatorGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(elevatorVelocity.getValueAsDouble()) / kElevatorGearRatio;
    inputs.appliedVolts = elevatorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {elevatorCurrent.getValueAsDouble()};
    inputs.bottomLimit = m_elevatorStop.get();
  }

  /**
   * Set the position of the elevator based on the coral mech's position above ground
   *
   * <p>This method requests FOC control of the Motion Magic, but if the motor is unlicensed, the
   * controller will default to non-FOC mode.
   *
   * @param position The height of the coral mechanism above the ground
   */
  @Override
  public void setPosistion(Distance position) {

    // This is the conversion from Elevator height in inches to motor rotations
    // (commanded position - zero_height) /  sproket_radius * gear ratio / 2π
    m_commandedMotorPosition =
        position
            .minus(kElevatorZeroHeight) // Delta y
            .div(kElevatorSproketRadius) // into angle
            .times(kElevatorGearRatio) // output shaft to motor angle
            .times(Radians.of(1.0)); // this is in radians

    // Log the value and send the rotation position to the motor
    Logger.recordOutput("Mechanism/Elevator/CommandPos", m_commandedMotorPosition.in(Rotations));
    m_elevatorMotor.setControl(
        m_motionMagic.withPosition(m_commandedMotorPosition.in(Rotations)).withEnableFOC(true));
  }

  /** Stop the elevator */
  @Override
  public void stop() {
    m_elevatorMotor.stopMotor();
  }

  /**
   * Configure the PIDs and other constants
   *
   * @param Kg The gravity gain
   * @param Ks The static gain
   * @param Kv The feedforward veliocity gain
   * @param Ka The feedforward acceleration gain
   * @param Kp The proportional gain (PID)
   * @param Ki The integral gain (PID)
   * @param Kd The differential gain (PID)
   * @param velocity The standard elevator linear velocity in m/s
   * @param acceleration The standard elevator linear acceleration in m/s/s
   * @param jerk The standard elevator linear jerk in m/s/s/s
   */
  @Override
  public void configure(
      double Kg,
      double Ks,
      double Kv,
      double Ka,
      double Kp,
      double Ki,
      double Kd,
      LinearVelocity velocity,
      LinearAcceleration acceleration,
      double jerk) {

    elevatorConfig.Slot0.kG = Kg;
    elevatorConfig.Slot0.kS = Ks;
    elevatorConfig.Slot0.kV = Kv;
    elevatorConfig.Slot0.kA = Ka;
    elevatorConfig.Slot0.kP = Kp;
    elevatorConfig.Slot0.kI = Ki;
    elevatorConfig.Slot0.kD = Kd;

    // Angular Velocity and Acceleration for Motion Magic in rot/s and rot/s/s
    double angularVerlocity =
        velocity.in(MetersPerSecond)
            / kElevatorSproketRadius.in(Meters)
            * kElevatorGearRatio
            / (2 * Math.PI);
    double angularAcceleration =
        acceleration.in(MetersPerSecondPerSecond)
            / kElevatorSproketRadius.in(Meters)
            * kElevatorGearRatio
            / (2 * Math.PI);

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = angularVerlocity;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = angularAcceleration;
    elevatorConfig.MotionMagic.MotionMagicJerk = jerk;

    PhoenixUtil.tryUntilOk(5, () -> m_elevatorMotor.getConfigurator().apply(elevatorConfig, 0.25));
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
    m_elevatorMotor.setControl(new VoltageOut(volts));
  }

  /**
   * Return a boolean supplier of whether the elevator is at the requested height
   *
   * <p>Realtive tolerance is 1%
   *
   * <p>Absolute tolerance is 0.5 rotations
   */
  @Override
  public BooleanSupplier isAtPosition() {
    return () ->
        MiscFuncs.isclose(
            m_commandedMotorPosition.in(Rotations),
            m_elevatorMotor.getPosition().getValueAsDouble(),
            0.01,
            0.5);
  }

  @Override
  public double getMotorPosition() {
    return m_elevatorMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getCommandedPosition() {
    return m_commandedMotorPosition.in(Rotations);
  }

  /** Return the value of the bottom stop switch at the bottom of the elevator */
  @Override
  public boolean getBottomStop() {
    return m_elevatorStop.get();
  }

  /** Set the neutral mode of the elevator to COAST */
  @Override
  public void setCoast() {
    m_elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /** Set the neutral mode of the elevator to BRAKE */
  @Override
  public void setBrake() {
    m_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }
}
