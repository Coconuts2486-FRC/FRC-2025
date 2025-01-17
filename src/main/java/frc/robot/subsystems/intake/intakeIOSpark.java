package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.intake.intakeIO;

public class intakeIOSpark implements intakeIO{
  SparkMax intakeSpark = new SparkMax(deviceId:999, MotorType.kBrushless);

}
