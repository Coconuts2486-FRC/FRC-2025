package frc.robot.subsystems.LED;

import frc.robot.util.RBSISubsystem;

public class LED extends RBSISubsystem {
  private LEDIO io;

  public LED(LEDIO io) {
    this.io = io;
  }

  public void m_led() {
    io.m_animate();
  }
}
