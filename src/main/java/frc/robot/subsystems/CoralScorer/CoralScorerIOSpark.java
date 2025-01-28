package frc.robot.subsystems.CoralScorer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.CANandPowerPorts;

public class CoralScorerIOSpark implements CoralScorerIO {

  public CoralScorerIOSpark() {}

  private final SparkMax coralMotor =
      new SparkMax(CANandPowerPorts.Coral_Scorer.getDeviceNumber(), MotorType.kBrushless);

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
    coralMotor.set(velocity);
  }
}
