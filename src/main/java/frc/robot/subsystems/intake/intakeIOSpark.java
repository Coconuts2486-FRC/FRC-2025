package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.intakeIO;

public class intakeIOSpark implements intakeIO{
  private final SparkMax intakeSpark = new SparkMax (999, MotorType.kBrushless);
  private final SparkClosedLoopController intakeController = intakeSpark.getClosedLoopController();
  public intakeIOSpark(){

  }
  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    intakeController.setReference(Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec), ControlType.kMAXMotionVelocityControl);
  }
}
