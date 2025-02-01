package frc.robot.subsystems.intake;

public interface intakeIO {
  public static class intakeIOInputs {
    public double velocity = 0.0;
  }

  public final int[] powerPorts = {};

  // Stops the intake?
  public default void stopRollers() {}

  public default void stopPivot() {}

  // Sets the velocity
  public default void setIntakeVelocity(double velocityRadPerSec) {}

  // Sets the voltage for rollers
  public default void setRollerVolts(double volts) {}

  // Sets the voltage for the pivot on the intake
  public default void setPivotVolts(double volts) {}

  // Sets PID constants
  public default void configureIntakePID(double kP, double kI, double kD) {}
}
