package frc.robot.subsystems.elevator;

public interface elevatorIO {

  public final int[] powerPorts = {};

  public default void setPosistion(double posistion) {}

  public default void stop() {}

  public default void configure(
      double Kg,
      double Ks,
      double Kv,
      double Ka,
      double Kp,
      double Ki,
      double Kd,
      double velocity,
      double aceleration,
      double jerk) {}

  public default void setVoltage(double volts) {}
}
