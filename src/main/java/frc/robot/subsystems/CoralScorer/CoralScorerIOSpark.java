package frc.robot.subsystems.CoralScorer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralScorerIOSpark implements CoralScorerIO {
private SparkMax motor = new SparkMax(0, MotorType.kBrushless);
private SparkMaxConfig config = new SparkMaxConfig();
private SparkClosedLoopController m_pidController;

public CoralScorerIOSpark() {
  config.encoder.positionConversionFactor(1000);
  config.
}
@Override 
public void moveXInches(double inches) {
motor.configure(config, null, null);
motor.getEncoder().getPosition();
m_pidController.setReference(inches, SparkBase.ControlType.kSmartMotion);
}
  
}
