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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
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

  public AlgaeMech(AlgaeMechIO io) {
    this.io = io;

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeMech", inputs);
    Logger.recordOutput("Overrides/AlgaeMechPivot", !disableOverride.getAsBoolean());

    // Check if disabled
    if (disableSupplier.getAsBoolean()) {
      stop();
      LED.getInstance().algaemechEstopped =
          disableSupplier.getAsBoolean() && DriverStation.isEnabled();
    }
  }

  public void stop() {
    io.stop();
  }

  public void pivotToPosition(double position) {
    io.pivotToPosition(position);
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysId.quasistatic(direction);
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysId.dynamic(direction);
  // }

  // Pivot to stored position
  public void pivotUp() {
    io.pivotToPosition(.209);
  }

  // Pivots to position to pick up off floor
  public void pivotHorizontal() {
    // io.pivotToPosition(.45);
    io.pivotToPosition(.45);
  }

  // Pivots to remove coral from reef
  public void pivotOffReef() {
    io.pivotToPosition(.521);
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
