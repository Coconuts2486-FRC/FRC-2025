package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralScorer.CoralScorer;

public class CoralScorerCommand extends Command {
  private final CoralScorer coralScorer;

  public CoralScorerCommand(CoralScorer coralScorer) {
    this.coralScorer = coralScorer;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    coralScorer.stop();
  }
}
