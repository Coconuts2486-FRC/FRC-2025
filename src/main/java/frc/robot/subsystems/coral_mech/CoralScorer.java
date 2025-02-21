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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LED.LED;
import frc.robot.util.RBSISubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class CoralScorer extends RBSISubsystem {
  private final CoralScorerIO io;
  private final CoralScorerIOInputsAutoLogged inputs = new CoralScorerIOInputsAutoLogged();
  private int delaySetup = 1;
  private Timer timer = new Timer();

  private boolean hasCoral = false;

  /** Constructor */
  public CoralScorer(CoralScorerIO io) {
    this.io = io;
    setDefaultCommand(Commands.run(() -> automaticIntake(), this));
  }

  /** Initialize the default command for this subsystem */
  public void initDefaultCommand() {}

  public boolean coralDelay(DoubleSupplier delay) {
    if (delaySetup == 1) {
      timer.reset();
      timer.start();
      delaySetup = 2;
    }
    if (delaySetup == 2) {
      if (timer.get() - delay.getAsDouble() >= 0) {
        delaySetup = 1;
        return true;
      }
    }
    return false;
  }

  /** Periodic function called every robot cycle */
  @Override
  public void periodic() {

    // Log the execution time
    long start = System.nanoTime();

    io.updateInputs(inputs);
    Logger.processInputs("CoralScorer", inputs);
    LED.setCoralReady(hasCoral);

    // Quick logging to see how long this periodic takes
    long finish = System.nanoTime();
    long timeElapsed = finish - start;
    Logger.recordOutput("LoggedRobot/CoralCodeMS", (double) timeElapsed / 1.e6);
  }

  public void runVolts(double volts) {
    io.setVolts(volts);
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void automaticIntake() {
    if (!io.getLightStop()) {

      io.setPercentOut(.2);
      hasCoral = true;
    } else {

      io.setPercentOut(0);
    }
  }

  public void setCoralPercent(double percent) {
    io.setPercentOut(percent);

    if (percent > 0) {
      hasCoral = false;
    }
  }

  public void stop() {
    io.stop();
  }

  public boolean getLightStop() {
    return io.getLightStop();
  }

  @Override
  public int[] getPowerPorts() {
    return io.powerPorts;
  }
}
