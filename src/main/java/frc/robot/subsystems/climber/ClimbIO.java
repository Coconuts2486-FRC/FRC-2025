package frc.robot.subsystems.climber;

public interface ClimbIO {

  public default void turnClimbServo(double position) {}

  public default void twistMotorToPosition(double positionInRotations) {}

  public default void configPID(double kP, double kI, double kD) {}

  public default void twistMotorVolts(double volts) {}
}
