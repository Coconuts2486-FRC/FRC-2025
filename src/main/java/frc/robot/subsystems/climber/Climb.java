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

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.RBSISubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Climber control class
 *
 * <p>This climber subsystem is based on the RBSI Subsystem, which allows for power monitoring in
 * addition to the underlying WPILib Subsystem functions.
 *
 * <p>This subsystem is based on the AdvantageKit model (https://docs.advantagekit.org/). This class
 * is the generic subsystem container that deals with outward-facing API calls (move to this
 * position, stop, etc.), while the IO files in this directory define the hardware- specific
 * function calls that implement the directives from the API.
 */
public class Climb extends RBSISubsystem {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;

  /** Constructor */
  public Climb(ClimbIO io) {
    this.io = io;
    // NOTE: At the moment, only PID are actually used!
    io.configPID(kSReal, kVReal, kAReal, kPReal, kIReal, kDReal, kVelocity, kAcceleration);

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).div(Seconds.of(1.5)), // QuasiStatis
                Volts.of(1.5), // Dynamic
                Seconds.of(2.0),
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Units.Volts)), null, this));
  }

  /** Periodic function called every robot cycle */
  @Override
  public void periodic() {

    // Log the execution time
    long start = System.nanoTime();

    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    // Quick logging to see how long this periodic takes
    long finish = System.nanoTime();
    long timeElapsed = finish - start;
    Logger.recordOutput("LoggedRobot/ClimbCodeMS", (double) timeElapsed / 1.e6);
  }

  /**
   * Twist the climb mechanism to a specified position
   *
   * @param position The position to which to move the climb
   */
  public void twistToPosition(double position) {
    io.twistMotorToPosition(position);
  }

  /**
   * Move the mechanism at a specified power until it reaches a specified position
   *
   * @param percent The percent output at which to run the motor
   * @param position The position to which to move the climb
   */
  public void goUntilPosition(double percent, double position) {
    if (io.getEncoderPose() > position) {
      io.setMotorPercent(0);
    } else {
      io.setMotorPercent(percent);
    }
  }

  /**
   * Manually control the climb mechanism
   *
   * @param controlVolts The voltage at which to run the climb motor
   */
  public void manualControl(double controlVolts) {
    io.twistMotorVolts(controlVolts * 3);
  }

  /**
   * Toggle the ratchet mechanism servo for the climber
   *
   * @param position The servo position for the ratchet toggle
   */
  public void rachetToggle(double position) {
    io.turnClimbServo(position);
  }

  /** Stop the mechanism */
  public void stop() {
    io.setMotorPercent(0);
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

  /** Return the power ports used by this mechanism */
  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
