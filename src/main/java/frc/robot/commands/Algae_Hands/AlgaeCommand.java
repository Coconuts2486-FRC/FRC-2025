package frc.robot.commands.Algae_Hands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae_hands.AlgaeHands;

public class AlgaeCommand extends Command {
  private final AlgaeHands handsOfAlgae;

  public AlgaeCommand(AlgaeHands handsOfAlgaeHands) {
    this.handsOfAlgae = handsOfAlgaeHands;
    addRequirements(handsOfAlgaeHands);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    handsOfAlgae.runVelocity(4);
  }

  @Override
  public void end(boolean interrupted) {
    handsOfAlgae.stop();
  }
}
