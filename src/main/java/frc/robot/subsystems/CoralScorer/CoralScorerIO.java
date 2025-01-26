package frc.robot.subsystems.CoralScorer;

import org.littletonrobotics.junction.AutoLog;

public interface CoralScorerIO {

  @AutoLog
  public static class IntakeIOInputs {}

  public default void setVolts(double volts) {}

  public default void stop() {}
}
