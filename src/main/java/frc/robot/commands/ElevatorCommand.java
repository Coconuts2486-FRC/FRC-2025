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

package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;

public class ElevatorCommand extends Command {

  private final Distance position;
  private final LinearAcceleration acceleration;
  private final LinearVelocity velocity;
  private final Elevator elevator;

  public ElevatorCommand(
      Supplier<Distance> position,
      LinearAcceleration acceleration,
      LinearVelocity velocity,
      Elevator elevator) {
    this.position = position.get();
    this.acceleration = acceleration;
    this.velocity = velocity;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {

    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        elevator.configure(
            kGReal, kSReal, kVReal, kAReal, kPReal, kIReal, kDReal, velocity, acceleration, 0.);
        break;
      case SIM:
        elevator.configure(
            kGSim, kSSim, kVSim, kASim, kPSim, kISim, kDSim, velocity, acceleration, 0.);
        break;
    }
  }

  @Override
  public void execute() {

    elevator.setPosistion(position);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    System.out.println("Done");
  }
}
