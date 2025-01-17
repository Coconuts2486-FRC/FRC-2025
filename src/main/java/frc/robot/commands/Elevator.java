package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.elevator;

public class Elevator extends Command {

  private elevator elevator;

  public Elevator(elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    elevator.configure(0, 0, 0, 0, 0, 0, 0, 1.4, 2.4, 0);
  }

  @Override
  public void execute() {
    elevator.setPosistion(1);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }
}
