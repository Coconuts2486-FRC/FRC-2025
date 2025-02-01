package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralScorer.CoralScorer;

public class CoralScorerCommand extends Command {

  private final CoralScorer haberdashery;

  private final double volts;

  public CoralScorerCommand(CoralScorer haberdashery, double volts) {
    this.haberdashery = haberdashery;
    this.volts = volts;
    addRequirements(haberdashery);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    haberdashery.runVoltsWithLS(volts);
  }

  @Override
  public void end(boolean interupted) {
    haberdashery.stop();
  }
}
