package frc.robot.subsystems.CoralScorer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.BooleanSupplier;

public class CoralScorerIOSpark implements CoralScorerIO {

  public CoralScorerIOSpark() {}

  private final SparkMax coralMotor = new SparkMax(16, MotorType.kBrushless);

  private final DigitalInput lightStop = new DigitalInput(1);

  @Override
  public void setVolts(double volts) {
    coralMotor.setVoltage(volts);
  }

  @Override
  public void lightStop(BooleanSupplier lightStop) {
    if (lightStop.getAsBoolean()) {}
  }

  @Override
  public boolean getLightStop() {
    return lightStop.get();
  }

  @Override
  public void stop() {
    coralMotor.stopMotor();
  }
}
