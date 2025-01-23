package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeCommand extends Command {

  private final Intake intake;

  public IntakeCommand(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double encodervalue = intake.getEncoder();

    System.out.println(encodervalue);
  }

  @Override
  public void end(boolean interrupted) {}
}
