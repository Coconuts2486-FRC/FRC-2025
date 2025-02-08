package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral_mech.CoralScorer;

public class CoralScorerCommand extends Command {
  private final CoralScorer coralScorer;
  private final double velocity;

  public CoralScorerCommand(CoralScorer coralScorer, double velocity) {
    this.coralScorer = coralScorer;
    this.velocity = velocity;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coralScorer.setVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    coralScorer.stop();
  }
}
