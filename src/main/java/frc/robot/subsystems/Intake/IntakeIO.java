package frc.robot.subsystems.intake;

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

  public default void rollerSpeed(double speed) {}

  public default void stop() {}

  public default void setPivotVolts(double volts) {}

  public default void configure(double kP, double kI, double kD) {}

  public default void setRollerVolts(double volts) {}

  public default double getEncoder() {
    return 0.0;
  }
}
