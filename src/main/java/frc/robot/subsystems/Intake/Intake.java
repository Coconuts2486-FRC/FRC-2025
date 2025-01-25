package frc.robot.subsystems.Intake;

import frc.robot.util.RBSISubsystem;

public class Intake extends RBSISubsystem {
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void runPivotVolts(double volts) {
    io.setPivotVolts(volts);
  }

  public void runIntakeRollers(double speed) {
    io.runIntakeRollers(speed);
  }

  public void setRollerVolts(double volts) {
    io.setRollerVolts(volts);
  }

  public double getEncoder() {
    return io.getEncoder();
  }

  public void stop() {
    io.stop();
  }
}
