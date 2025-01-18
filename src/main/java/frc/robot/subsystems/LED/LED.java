package frc.robot.subsystems.LED;

import frc.robot.util.RBSISubsystem;

public class LED extends RBSISubsystem {
  private LEDIO io;

  public LED(LEDIO io) {
    this.io = io;
  }

  public void rainbowTwinkle() {
    io.rainbowTwinkle();
  }

  public void off() {
    io.off();
  }

  public void larson() {
    io.larson();
  }

  public void police() {
    io.police();
  }
}
