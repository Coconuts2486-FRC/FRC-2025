package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;

public class intakeIOSpark implements intakeIO {
  // Need to figure out how to code in the encoder
  
  private final SparkMax intakeSpark1 = new SparkMax(21, MotorType.kBrushless); // Roller motor
  private final SparkMax intakeSpark2 = new SparkMax(22, MotorType.kBrushless); // Pivot motor
  private final SparkClosedLoopController intakeController1 =
      intakeSpark1.getClosedLoopController();
  private final SparkClosedLoopController intakeController2 =
      intakeSpark2.getClosedLoopController();

  // public final int[] powerPorts = {
  //   CANandPowerPorts.INTAKE_LEADER.getPowerPort(),
  //   CANandPowerPorts.INTAKE_FOLLOWER.getPowerPort()
  // };
  public intakeIOSpark() {}

  @Override
  public void stopRollers() {
    intakeSpark1.stopMotor();
    intakeSpark2.stopMotor();
  }

  @Override
  public void setIntakeVelocity(double velocityRadPerSec) {
    intakeController1.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        ControlType.kMAXMotionVelocityControl);
    intakeController2.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        ControlType.kMAXMotionVelocityControl);
  }
  @Override 
  public void setPivotVolts(double volts) {
    intakeSpark2.setVoltage(volts);
  }
  @Override
  public void setRollerVolts(double volts) {
    intakeSpark1.setVoltage(volts);
  }

  @Override
  public void configureIntakePID(double kP, double kI, double kD) {}
}
