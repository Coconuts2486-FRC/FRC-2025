package frc.robot.subsystems.CoralScorer;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.CANandPowerPorts;

public class CoralScorerIOTalon implements CoralScorerIO {

  public CoralScorerIOTalon() {}

  private final TalonFX coralMotor = new TalonFX(CANandPowerPorts.Coral_Scorer.getDeviceNumber());

  public final int[] powerPorts = {
    CANandPowerPorts.Coral_Scorer.getPowerPort(), CANandPowerPorts.Coral_Scorer.getPowerPort()
  };

  @Override
  public void setVolts(double volts) {
    coralMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    coralMotor.stopMotor();
  }

  @Override
  public void setVelocity(double velocity) {
    velocity = velocity * -1;
    coralMotor.set(velocity);
  }
}
