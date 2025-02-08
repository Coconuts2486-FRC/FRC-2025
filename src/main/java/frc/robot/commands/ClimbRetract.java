package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climb;

public class ClimbRetract extends Command {
  private final Climb climb;

  public ClimbRetract(Climb climb) {
    this.climb = climb;
  }

  @Override
  public void initialize() {
    climb.rachetToggle(true);
  }

  @Override
  public void execute() {
    climb.twistToPosition(0);
  }

  @Override
  public void end(boolean interrupted) {}
}
