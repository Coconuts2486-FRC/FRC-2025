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

  public void setRollerVolts(double volts) {
    io.setRollerVolts(volts);
  }

  public void stop() {
    io.stop();
  }

  public void configure(
      double kG,
      double kS,
      double kV,
      double kA,
      double kP,
      double kI,
      double kD,
      double velocity,
      double acceleration,
      double jerk) {}
}
