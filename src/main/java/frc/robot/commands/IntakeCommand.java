package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeCommand extends Command {

  private final Intake intake;
  private final double direction;

  public IntakeCommand(Intake intake, double direction) {
    this.intake = intake;
    this.direction = direction;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setRollerVolts(-direction);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
