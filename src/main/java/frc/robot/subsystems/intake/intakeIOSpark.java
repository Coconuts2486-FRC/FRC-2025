package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class intakeIOSpark implements intakeIO {
  SparkMax intakeSpark = new SparkMax(999, MotorType.kBrushless);
}
