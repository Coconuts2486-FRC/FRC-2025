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

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;

/** Climb hardware class for TalonFX */
public class ClimbIOTalonFX implements ClimbIO {
  private final Servo ratchetServo = new Servo(CANandPowerPorts.CLIMB_SERVO);
  private final TalonFX m_climbMotor = new TalonFX(CANandPowerPorts.CLIMB.getDeviceNumber());
  private final DutyCycleEncoder m_climbEncoder =
      new DutyCycleEncoder(CANandPowerPorts.CLIMB_PIVOT_ENCODER);
  private final TalonFXConfiguration climbConfig = new TalonFXConfiguration();

  // Power port
  public final int[] powerPorts = {CANandPowerPorts.CLIMB.getPowerPort()};

  // Set up the Motion Magic instance
  // private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

  /** Constructor */
  public ClimbIOTalonFX() {
    // Set motor configurations
    climbConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Neutral mode is BRAKE
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbConfig.Slot0 = new Slot0Configs().withKP(kPReal).withKI(kIReal).withKD(kDReal);
    PhoenixUtil.tryUntilOk(5, () -> m_climbMotor.getConfigurator().apply(climbConfig, 0.25));
  }

  /**
   * Set the position of the ratchet servo
   *
   * @param position The position to which to set the ratchet servo
   */
  @Override
  public void turnClimbServo(double position) {
    ratchetServo.set(position);
  }

  /**
   * Twist the motor to the specified position
   *
   * <p>This method uses MotionMagic rather than base PID control
   *
   * @param position The position to which to set the motor
   */
  @Override
  public void twistMotorToPosition(double position) {
    PIDController pid = new PIDController(kPReal, kIReal, kDReal);
    m_climbMotor.setControl(new DutyCycleOut(-pid.calculate(m_climbEncoder.get(), position)));
    // m_climbMotor.setControl(m_motionMagic.withPosition(position).withEnableFOC(true));
  }

  /** Get the climb encoder pose */
  @Override
  public double getEncoderPose() {
    return m_climbEncoder.get();
  }

  /**
   * Run the motor to the specified twist voltage
   *
   * @param volts The voltage to which to set the motor
   */
  @Override
  public void twistMotorVolts(double volts) {
    m_climbMotor.setVoltage(volts);
  }

  /**
   * Run the motor at the specified duty cycle percentage
   *
   * @param percent The duty cycle percent at which to run the motor
   */
  @Override
  public void setMotorPercent(double percent) {
    m_climbMotor.setControl(new DutyCycleOut(percent));
  }

  /**
   * Configure the PIDs and other constants
   *
   * @param Ks The static gain
   * @param Kv The feedforward veliocity gain
   * @param Ka The feedforward acceleration gain
   * @param Kp The proportional gain (PID)
   * @param Ki The integral gain (PID)
   * @param Kd The differential gain (PID)
   * @param velocity The climb mechanism angular velocity in rot/s
   * @param acceleration The climb mechanism angular acceleration in rot/s/s
   */
  @Override
  public void configPID(
      double Ks,
      double Kv,
      double Ka,
      double Kp,
      double Ki,
      double Kd,
      AngularVelocity velocity,
      AngularAcceleration acceleration) {
    // Update the Slot0 configuration
    climbConfig.Slot0 = climbConfig.Slot0.withKP(Kp).withKI(Ki).withKD(Kd);

    // climbConfig.MotionMagic.MotionMagicCruiseVelocity = angularVerlocity;
    // climbConfig.MotionMagic.MotionMagicAcceleration = angularAcceleration;

    PhoenixUtil.tryUntilOk(5, () -> m_climbMotor.getConfigurator().apply(climbConfig, 0.25));
  }

  /** Return the list of PDH power ports used for this mechanism */
  @Override
  public int[] getPowerPorts() {
    return powerPorts;
  }
}
