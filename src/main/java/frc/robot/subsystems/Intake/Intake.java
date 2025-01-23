package frc.robot.subsystems.Intake;

import frc.robot.util.RBSISubsystem;

public class Intake extends RBSISubsystem {
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void runIntakeRollers(double speed) {
    io.runIntakeRollers(speed);
  }

  public double getEncoder() {
    return io.getEncoder();
  }
}
