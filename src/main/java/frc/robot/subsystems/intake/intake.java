package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class intake {
  private final intakeIO io;

    public intake (intakeIO io){
      this.io = io;
    }

  public void stop() {
    io.stopIntake();
  }
  public void runVolts(double volts) {
    io.setIntakeVolts(volts);
  }
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setIntakeVelocity(velocityRadPerSec);
  }
}
