package frc.robot.subsystems.intake;

  public interface intakeIO {
    public static class intakeIOInputs{
      public double velocity = 0.0;

    
    }
    public default void setVelocity(double velocityRadPerSec, double ffVolts) {}
  }

