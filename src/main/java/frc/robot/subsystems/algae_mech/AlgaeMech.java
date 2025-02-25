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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LED.LED;
import frc.robot.util.RBSISubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AlgaeMech extends RBSISubsystem {

  private final AlgaeMechIO io;
  private final AlgaeMechIOInputsAutoLogged inputs = new AlgaeMechIOInputsAutoLogged();
  // private final SysIdRoutine sysId;

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier disableOverride;
  private boolean toggleStow = true;
  private int pivotIndex = 3;

  /** Constructor */
  public AlgaeMech(AlgaeMechIO io) {
    this.io = io;
    setDefaultCommand(Commands.run(() -> cyclePositions(), this));

    //   // Configure SysId
    //   sysId =
    //       new SysIdRoutine(
    //           new SysIdRoutine.Config(
    //               Volts.of(1.0).div(Seconds.of(1.5)), // QuasiStatis
    //               Volts.of(1.5), // Dynamic
    //               Seconds.of(2.0),
    //               (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
    //           new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  /** Set the override for this subsystem */
  public void setOverrides(BooleanSupplier disableOverride) {
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
    this.disableOverride = disableOverride;
  }

  /** Initialize the default command for this subsystem */

  /** Periodic function called every robot cycle */
  @Override
  public void periodic() {
    // Log the execution time
    long start = System.nanoTime();

    io.updateInputs(inputs);
    Logger.processInputs("AlgaeMech", inputs);
    Logger.recordOutput("Overrides/AlgaeMechPivot", !disableOverride.getAsBoolean());

    // Check if disabled
    if (disableOverride.getAsBoolean()) {
      stop();
      LED.setAgaeMechEStop(disableOverride.getAsBoolean() && DriverStation.isEnabled());
    }

    // Quick logging to see how long this periodic takes
    long finish = System.nanoTime();
    long timeElapsed = finish - start;
    Logger.recordOutput("LoggedRobot/AlgaeCodeMS", (double) timeElapsed / 1.e6);
    fixIndex();
  }

  public void stop() {
    io.stop();
  }

  public void pivotToPosition(double position) {
    if (!disableSupplier.getAsBoolean()) {
      io.pivotToPosition(position);
    }
  }

  public void setPercent(double percent) {
    if (!disableSupplier.getAsBoolean()) {
      io.setPercent(percent);
    }
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysId.quasistatic(direction);
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysId.dynamic(direction);
  // }

  // Pivot to stored position
  public void toggleUp(boolean stow) {
    if (!disableSupplier.getAsBoolean()) {
      this.toggleStow = stow;
    }
  }

  public boolean getToggleStow() {
    return toggleStow;
  }

  public void holdToggle() {
    if (!disableSupplier.getAsBoolean()) {
      if (toggleStow == true) {
        io.pivotToPosition(.209);
      } else {
        io.pivotToPosition(.35);
      }
    }
  }

  // Toggles between the three pivot positions
  // <<<
  public void indexPoseUp() {
    pivotIndex = pivotIndex + 1;
  }

  public void indexPoseDown() {
    pivotIndex = pivotIndex - 1;
    if (pivotIndex < 1) {}
  }

  private void fixIndex() {
    if (pivotIndex < 1) {
      pivotIndex = 1;
    }
    if (pivotIndex > 3) {
      pivotIndex = 3;
    }
  }

  public void setIndexPose(int index) {
    pivotIndex = index;
  }

  public void cyclePositions() {
    if (!disableSupplier.getAsBoolean()) {
      switch (pivotIndex) {
        case 3:
          io.pivotToPosition(.209);
          break;
        case 2:
          io.pivotToPosition(.35);
          break;
        case 1:
          io.pivotToPosition(.45);
          break;
        default:
          io.pivotToPosition(.35);
      }
    }
  }

  // >>>

  public void pivotUp() {
    if (!disableSupplier.getAsBoolean()) {
      io.pivotToPosition(.209);
    }
  }

  // Pivots to position to pick up off floor
  public void pivotHorizontal() {
    if (!disableSupplier.getAsBoolean()) {
      // io.pivotToPosition(.45);
      io.pivotToPosition(.35);
    }
  }

  // Pivots to remove algae from reef
  public void pivotOffReef() {
    if (!disableSupplier.getAsBoolean()) {
      io.pivotToPosition(.521);
    }
  }

  // /** Returns the current velocity in RPM. */
  // @AutoLogOutput(key = "Mechanism/Flywheel")
  // //public double getVelocityRPM() {
  //   //  return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  // }

  @Override
  public int[] getPowerPorts() {
    return io.powerPorts;
  }
}
