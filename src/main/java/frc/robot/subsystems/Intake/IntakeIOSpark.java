package frc.robot.subsystems.Intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax intakeRoller = new SparkMax(21, MotorType.kBrushless);

  private final SparkMax intakePivot = new SparkMax(22, MotorType.kBrushless);
  private final SparkClosedLoopController rollerPid = intakeRoller.getClosedLoopController();

  private final AbsoluteEncoder encoder = intakePivot.getAbsoluteEncoder();

  public IntakeIOSpark() {}

  @Override
  public void runIntakeRollers(double speedRPS) {
    rollerPid.setReference(1, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public double getRollerCurrent() {
    return intakeRoller.getOutputCurrent();
  }

  @Override
  public double getEncoder() {
    return encoder.getPosition();
  }

  @Override
  public void setPivotVolts(double volts) {
    intakePivot.setVoltage(volts);
  }

  @Override
  public void setRollerVolts(double volts) {
    intakeRoller.setVoltage(volts);
  }

  @Override
  public void stop() {
    intakeRoller.stopMotor();
    intakePivot.stopMotor();
  }
}
