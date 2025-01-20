package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
 @AutoLog
  public static class IntakeIOInputs {

  }
  
  public default void pivotToPosition(double position) {}

  //Runs the intake rollers in radians per second
  public default void runIntakeRollers(double speedRPS) {}

    //Stop in open loop.
    public default void stop() {}

    // Set velocity PID constants.
    public default void configurePID(double kP, double kI, double kD) {}

    public default double getRollerCurrent() {
      return 0.0;
    }
}