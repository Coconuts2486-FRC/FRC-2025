package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeCommand extends Command {

  private final Intake intake;
  private final double wantedPosistion;
  private final double rollerSpeed;
  private final double test; // get rid of this variable in future

  public IntakeCommand(Intake intake, double wantedPosistion, double rollerSpeed, double test) {
    this.intake = intake;
    this.wantedPosistion = wantedPosistion;
    this.rollerSpeed = rollerSpeed;
    this.test = test;
  }

  @Override
  public void initialize() {
    intake.configure(1, 0, 0);
  }

  @Override
  public void execute() {
    if (test == 0) {
      intake.setPivotPosition(wantedPosistion);
      intake.rollerSpeed(rollerSpeed);
    } else if (test == 1) {
      System.out.println(intake.getEncoder());
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
