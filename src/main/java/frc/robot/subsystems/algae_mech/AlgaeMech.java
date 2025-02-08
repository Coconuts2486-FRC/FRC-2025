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
import frc.robot.subsystems.LED.LED;
import frc.robot.util.RBSISubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AlgaeMech extends RBSISubsystem {

  private final AlgaeMechIO io;
  private final AlgaeMechIOInputsAutoLogged inputs = new AlgaeMechIOInputsAutoLogged();

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier disableOverride;

  public AlgaeMech(AlgaeMechIO io) {
    this.io = io;
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

  @Override
  public int[] getPowerPorts() {
    return io.powerPorts;
  }
}
