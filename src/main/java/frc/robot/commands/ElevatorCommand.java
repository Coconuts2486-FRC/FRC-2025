package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {

  private final double posistion;
  private final double acceleration;
  private final double velocity;
  private final Elevator elevator;

  public ElevatorCommand(
      double posistion, double acceleration, double velocity, Elevator elevator) {
    this.posistion = posistion;
    this.elevator = elevator;
    this.acceleration = acceleration;
    this.velocity = velocity;
  }

  @Override
  public void initialize() {

    elevator.configure(0.3375, 0.075, 0.18629, 0.01, 18, 0, 0.01, velocity, acceleration, 0);
  }

  @Override
  public void execute() {
    double rotationsPosition = (35 / 54.75) * posistion - 10.86758;
    elevator.setPosistion(rotationsPosition);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }
}
