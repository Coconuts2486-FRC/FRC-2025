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

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {

  private final Intake intake;
  private final double wantedPosistion;
  private final double rollerSpeed;

  public IntakeCommand(Intake intake, double wantedPosistion, double rollerSpeed) {
    this.intake = intake;
    this.wantedPosistion = wantedPosistion;
    this.rollerSpeed = rollerSpeed;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.configure(1, 0, 0);
  }

  @Override
  public void execute() {

    intake.setPivotPosition(wantedPosistion);
    intake.rollerSpeed(rollerSpeed);

    // System.out.println(intake.getEncoder());

  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
