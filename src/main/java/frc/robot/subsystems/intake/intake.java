package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class intake {
  private final intakeIO io;

  public intake(intakeIO io) {
    this.io = io;
  }

  public void stopRollers() {
    io.stopRollers();
  }

  public void stopPivot() {
    io.stopPivot();
  }

  public void runRollerVolts(double volts) {
    io.setRollerVolts(volts);
  }

  public void runPivotVolts(double volts) {
    io.setPivotVolts(volts);
  }

  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setIntakeVelocity(velocityRadPerSec);
  }
}
