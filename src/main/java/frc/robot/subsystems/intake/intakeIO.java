package frc.robot.subsystems.intake;


public interface intakeIO {
  public static class intakeIOInputs {
    public double velocity = 0.0;
  }
  public final int[] powerPorts = {};
  // Stops the intake?
  public default void stopIntake(){}
  // Sets the velocity
  public default void setIntakeVelocity(double velocityRadPerSec) {}
  //Sets the voltage
  public default void setIntakeVolts(double volts) {}
  // Sets PID constants
  public default void configureIntakePID (double kP, double kI , double kD) {}
}
