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

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends RBSISubsystem {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Intake(IntakeIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).div(Seconds.of(1.5)), // QuasiStatis
                Volts.of(.75), // Dynamic
                Seconds.of(1.9),
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runPivotVolts(voltage.in(Units.Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setPivotPosition(double position) {
    io.setPivotPosition(position);
  }

  public void rollerSpeed(double speed) {
    io.rollerSpeed(speed);
  }

  public void runPivotVolts(double volts) {
    io.setPivotVolts(volts);
  }

  public void setRollerVolts(double volts) {
    io.setRollerVolts(volts);
  }

  public void stop() {
    io.stop();
  }

  public void configure(double kP, double kI, double kD) {
    io.configure(kP, kI, kD);
  }
  ;

  public double getEncoder() {
    return io.getEncoder();
  }

  @Override
  public int[] getPowerPorts() {
    return io.powerPorts;
  }
}
