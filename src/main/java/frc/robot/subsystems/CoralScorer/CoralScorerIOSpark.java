package frc.robot.subsystems.CoralScorer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralScorerIOSpark implements CoralScorerIO {

  private final SparkMax coralMotor = new SparkMax(16, MotorType.kBrushless);

  @Override
  public void setVolts(double volts) {
    coralMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    coralMotor.stopMotor();
  }
}
