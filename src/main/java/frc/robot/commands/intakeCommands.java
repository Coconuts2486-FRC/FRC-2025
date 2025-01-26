package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.intake;

public class intakeCommands extends Command {
  private final double volts;

  public intakeCommand(double volts) {
    this.volts = volts;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double intakeVoltage = volts;
  }

  @Override
  public void stopIntake() {
    intake.stop();
  }
}
