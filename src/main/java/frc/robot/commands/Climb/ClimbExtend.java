package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.Climb;

public class ClimbExtend extends Command {
  private final Climb climb;

  public ClimbExtend(Climb climb) {
    this.climb = climb;
  }

  @Override
  public void initialize() {
    climb.rachetToggle(false);
  }

  @Override
  public void execute() {
    climb.twistToPosition(180);
  }

  @Override
  public void end(boolean interrupted) {}
}
