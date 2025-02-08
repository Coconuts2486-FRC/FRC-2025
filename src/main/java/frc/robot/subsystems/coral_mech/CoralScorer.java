package frc.robot.subsystems.coral_mech;

import frc.robot.util.RBSISubsystem;

public class CoralScorer extends RBSISubsystem {
  private final CoralScorerIO io;

  public CoralScorer(CoralScorerIO io) {
    this.io = io;
  }

  public void runVolts(double volts) {
    io.setVolts(volts);
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void stop() {
    io.stop();
  }

  @Override
  public int[] getPowerPorts() {
    return io.powerPorts;
  }
}
