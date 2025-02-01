package frc.robot.subsystems.CoralScorer;

import frc.robot.util.RBSISubsystem;

public class CoralScorer extends RBSISubsystem {

  private final CoralScorerIO io;

  public CoralScorer(CoralScorerIO io) {
    this.io = io;
  }

  /** Stop in open loop. */
  public void stop() {
    io.stop();
  }

  public void runVolts(double volts) {
    io.setVolts(3 * volts);
  }

  public void runVoltsWithLS(double volts) {
    if (io.getLightStop() == false) {
      io.setVolts(3 * volts);
    } else {
      io.setVolts(-1.5 + (3 * volts));
    }
  }
}
