package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  public final int[] powerPorts = {};

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPivotPosition(double position) {}

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
