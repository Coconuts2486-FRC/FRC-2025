package frc.robot.subsystems.coral_mech;

import org.littletonrobotics.junction.AutoLog;

public interface CoralScorerIO {

  @AutoLog
  public static class IntakeIOInputs {}

  public final int[] powerPorts = {};

  public default void setVolts(double volts) {}

  public default void stop() {}

  public default void setVelocity(double velocity) {}
}
