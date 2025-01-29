package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {}

  public default void pivotToPosition(double position) {}

  public default void stop() {}

  public default void setPivotVolts(double volts) {}

  public default void setRollerVolts(double volts) {}

  public default void configure(
      double kG,
      double kS,
      double kV,
      double kA,
      double kP,
      double kI,
      double kD,
      double velocity,
      double acceleration,
      double jerk) {}
}
