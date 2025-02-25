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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.LED.LED;
import frc.robot.util.RBSISubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Elevator control class
 *
 * <p>This elevator subsystem is based on the RBSI Subsystem, which allows for power monitoring in
 * addition to the underlying WPILib Subsystem functions.
 *
 * <p>This subsystem is based on the AdvantageKit model (https://docs.advantagekit.org/). This class
 * is the generic subsystem container that deals with outward-facing API calls (move to this
 * position, stop, etc.), while the IO files in this directory define the hardware- specific
 * function calls that implement the directives from the API.
 */
public class Elevator extends RBSISubsystem {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier disableOverride;

  /** Constructor */
  public Elevator(ElevatorIO io) {

    this.io = io;

    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        io.configure(
            kGReal,
            kSReal,
            kVReal,
            kAReal,
            kPReal,
            kIReal,
            kDReal,
            kVelocity,
            kAcceleration,
            kJerk);
        break;
      case SIM:
        io.configure(
            kGSim, kSSim, kVSim, kASim, kPSim, kISim, kDSim, kVelocity, kAcceleration, kJerk);
        break;
      default:
        io.configure(0, 0, 0, 0, 0, 0, 0, MetersPerSecond.of(0), MetersPerSecondPerSecond.of(0), 0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).div(Seconds.of(1.5)), // QuasiStatis
                Volts.of(1.5), // Dynamic
                Seconds.of(2.0),
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Units.Volts)), null, this));

    setDefaultCommand(
        new ElevatorCommand(
            () -> ElevatorConstants.kElevatorZeroHeight,
            ElevatorConstants.kAcceleration.div(2), // Go slower on the way down
            ElevatorConstants.kVelocity.div(2), // Go slower on the way down
            this));
  }

  /** Set the override for this subsystem */
  public void setOverrides(BooleanSupplier disableOverride) {
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
    this.disableOverride = disableOverride;
  }

  /** Periodic function called every robot cycle */
  @Override
  public void periodic() {
    // Log the execution time
    long start = System.nanoTime();

    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Overrides/Elevator", !disableOverride.getAsBoolean());
    Logger.recordOutput("Mechanism/Elevator/isAtPosition", isAtPosition().getAsBoolean());
    Logger.recordOutput("Mechanism/Elevator/ElevatorMotorPosition", io.getMotorPosition());
    Logger.recordOutput("Mechanism/Elevator/ElevatorCommandedPosition", io.getCommandedPosition());

    // Check if disabled
    if (disableOverride.getAsBoolean()) {
      stop();
      setCoast();
      LED.setElevatorEStop(disableOverride.getAsBoolean() && DriverStation.isEnabled());
    }

    // Quick logging to see how long this periodic takes
    long finish = System.nanoTime();
    long timeElapsed = finish - start;
    Logger.recordOutput("LoggedRobot/ElevatorCodeMS", (double) timeElapsed / 1.e6);
  }

  /**
   * Set position
   *
   * <p>Does not run when Driver Station is disabled or override switch is thrown
   *
   * @param position The linear distance to which to move the elevator
   */
  public void setPosistion(Distance posistion) {
    if (!disableSupplier.getAsBoolean()) {
      io.setPosistion(posistion);
    }
  }

  /**
   * Run open loop at the specified voltage
   *
   * <p>Does not run when Driver Station is disabled or override switch is thrown
   *
   * @param volts The voltage at which to run the elevator
   */
  public void runVolts(double volts) {
    if (!disableSupplier.getAsBoolean()) {
      io.setVoltage(volts);
    }
  }

  /** Stop the elevator */
  public void stop() {
    io.stop();
  }

  /** Return a boolean supplier of whether the elevator is at the requested height */
  public BooleanSupplier isAtPosition() {
    return io.isAtPosition();
  }

  /* Configuaration and Setter / Getter Functions ************************** */
  public void configure(
      double Kg,
      double Ks,
      double Kv,
      double Ka,
      double Kp,
      double Ki,
      double Kd,
      LinearVelocity velocity,
      LinearAcceleration aceleration,
      double jerk) {
    io.configure(Kg, Ks, Kv, Ka, Kp, Ki, Kd, velocity, aceleration, jerk);
  }

  /** Return the bottom stop limit switch value */
  public boolean getBottomStop() {
    return io.getBottomStop();
  }

  /** Set the neutral mode for this mechanism to BRAKE */
  public void setCoast() {
    io.setCoast();
  }

  /** Set the neutral mode for this mechanism to BRAKE */
  public void setBrake() {
    io.setBrake();
  }

  /** Return the power ports used by this mechanism */
  @Override
  public int[] getPowerPorts() {
    return io.powerPorts;
  }

  /* SysId Functions ******************************************************* */
  // NOTE: This is how to measure the SysId for elevators that cannot hold
  //       themselves up under the force of gravity:
  // https://www.chiefdelphi.com/t/running-backwards-sysid-for-elevator-that-can-t-hold-itself-up/427876/2
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
