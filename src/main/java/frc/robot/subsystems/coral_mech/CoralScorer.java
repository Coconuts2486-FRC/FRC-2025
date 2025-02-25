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

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LED.LED;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Coral Scorer control class
 *
 * <p>This mechanism subsystem is based on the RBSI Subsystem, which allows for power monitoring in
 * addition to the underlying WPILib Subsystem functions.
 *
 * <p>This subsystem is based on the AdvantageKit model (https://docs.advantagekit.org/). This class
 * is the generic subsystem container that deals with outward-facing API calls (move to this
 * position, stop, etc.), while the IO files in this directory define the hardware- specific
 * function calls that implement the directives from the API.
 */
public class CoralScorer extends RBSISubsystem {
  private final CoralScorerIO io;
  private final CoralScorerIOInputsAutoLogged inputs = new CoralScorerIOInputsAutoLogged();

  private boolean hasCoral = false;

  /** Constructor */
  public CoralScorer(CoralScorerIO io) {
    this.io = io;

    setDefaultCommand(Commands.run(() -> automaticIntake(), this));
  }

  /** Periodic function called every robot cycle */
  @Override
  public void periodic() {

    // Log the execution time
    long start = System.nanoTime();

    io.updateInputs(inputs);
    Logger.processInputs("CoralScorer", inputs);
    LED.setCoralReady(hasCoral);

    Logger.recordOutput("Mechanism/CoralScorer/Velocity", io.getPercent());

    // Quick logging to see how long this periodic takes
    long finish = System.nanoTime();
    long timeElapsed = finish - start;
    Logger.recordOutput("LoggedRobot/CoralCodeMS", (double) timeElapsed / 1.e6);
  }

  /**
   * Run the mechanism at a specified voltage
   *
   * @param volts The voltage at which to run the mechanism
   */
  public void runVolts(double volts) {
    io.setVolts(volts);
  }

  /**
   * Run the motor at the specified velocity
   *
   * @param velocity The velocity (presumably in rot/sec) at which to run the motor
   */
  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  /**
   * Automatically intake the coral from the ramp to the mechanism
   *
   * <p>This is the default command action that runs unless interrupted
   */
  public void automaticIntake() {
    if (!io.getLightStop()) {

      io.setPercentOut(.16);
      hasCoral = true;
    } else {

      io.setPercentOut(0);
    }
  }

  /**
   * Set the coral rollers to run at a specified duty cycle percent
   *
   * @param percent The duty cycle percent at which to run the mechanism rollers
   */
  public void setCoralPercent(double percent) {
    io.setPercentOut(percent);

    if (percent > 0) {
      hasCoral = false;
    }
  }

  /** Stop the mechanism */
  public void stop() {
    io.stop();
  }

  /** Get the value of the light stop */
  public boolean getLightStop() {
    return io.getLightStop();
  }

  /** Return the power ports used by this mechanism */
  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
