package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface elevatorIO {

  public final int[] powerPorts = {};

  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void setPosistion(double posistion) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

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

  public default void limit(BooleanSupplier limitSwitch) {}

  public default void setCoast() {}

  public default void setBrake() {}
}
