package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeCommand extends Command {

  private final Intake intake;
  private final double wantedPosistion;

  public IntakeCommand(Intake intake, double wantedPosistion) {
    this.intake = intake;
    this.wantedPosistion = wantedPosistion;
  }

  @Override
  public void initialize() {
    intake.configure(1, 0, 0);
  }

  @Override
  public void execute() {
    intake.setPivotPosition(wantedPosistion);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
