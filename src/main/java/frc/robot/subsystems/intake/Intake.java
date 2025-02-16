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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.LED.LED;
import frc.robot.util.RBSISubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends RBSISubsystem {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier disableOverride;

  public Intake(IntakeIO io) {
    this.io = io;

    setDefaultCommand(Commands.run(() -> setPivotPosition(0.9), this));

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

  /** Set the override for this subsystem */
  public void setOverrides(BooleanSupplier disableOverride) {
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
    this.disableOverride = disableOverride;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Overrides/IntakePivot", !disableOverride.getAsBoolean());

    // Check if disabled
    if (disableOverride.getAsBoolean()) {
      stop();
      LED.setIntakeEStop(disableOverride.getAsBoolean() && DriverStation.isEnabled());
    }
  }

  /**
   * Set pivot position
   *
   * <p>Does not run when Driver Station is disabled or override switch is thrown
   *
   * @param position The position to which to move the intake pivot
   */
  public void setPivotPosition(double position) {
    if (!disableSupplier.getAsBoolean()) {
      io.setPivotPosition(position);
    }
  }

  /**
   * Set intake roller speed
   *
   * <p>Does not run when Driver Station is disabled or override switch is thrown
   *
   * @param speed The speed at which to run the intake rollers
   */
  public void rollerSpeed(double speed) {
    if (!disableSupplier.getAsBoolean()) {
      io.rollerSpeed(speed);
    }
  }

  /**
   * Run the intake pivot at a designated voltage
   *
   * <p>Does not run when Driver Station is disabled or override switch is thrown
   *
   * @param volts The voltage at which to run the intake pivot
   */
  public void runPivotVolts(double volts) {
    if (!disableSupplier.getAsBoolean()) {
      io.setPivotVolts(volts);
    }
  }

  /**
   * Run the intake rollers at a designated voltage
   *
   * <p>Does not run when Driver Station is disabled or override switch is thrown
   *
   * @param speed The voltage at which to run the intake rollers
   */
  public void setRollerVolts(double volts) {
    if (!disableSupplier.getAsBoolean()) {
      io.setRollerVolts(volts);
    }
  }

  public void stop() {
    io.stop();
  }

  /* Configuaration and Setter / Getter Functions ************************** */
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
