package frc.robot.subsystems.elevator;

import frc.robot.util.RBSISubsystem;

public class elevator extends RBSISubsystem {

  private final elevatorIO io;

  public elevator(elevatorIO io) {
    this.io = io;
  }

  public void setPosistion(double posistion) {
    io.setPosistion(posistion);
  }

  public void stop() {
    io.stop();
  }

  public void confiure(
      double Kg,
      double Ks,
      double Kv,
      double Ka,
      double Kp,
      double Ki,
      double Kd,
      double velocity,
      double aceleration,
      double jerk) {
    io.configure(Kg, Ks, Kv, Ka, Kp, Ki, Kd, velocity, aceleration, jerk);
  }
}
