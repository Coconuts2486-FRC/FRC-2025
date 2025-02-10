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

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {

  private final Distance position;
  private final AngularAcceleration acceleration;
  private final AngularVelocity velocity;
  private final Elevator elevator;

  public ElevatorCommand(
      Distance position,
      AngularAcceleration acceleration,
      AngularVelocity velocity,
      Elevator elevator) {
    this.position = position;
    this.acceleration = acceleration;
    this.velocity = velocity;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {

    // switch Constants.getMode(){
    //   case REAL:
    elevator.configure(kGReal, 0.075, 0.18629, 0.01, 18, 0, 0.01, velocity, acceleration, 0);
    // case SIM:
  }

  @Override
  public void execute() {

    elevator.setPosistion(position);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }
}
