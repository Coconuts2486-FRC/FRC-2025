package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX intakeRoller = new TalonFX(21);

  private final TalonFX intakePivot = new TalonFX(22);

  public IntakeIOKraken() {}

  @Override
  public void runIntakeRollers(double speedRPS) {
    // rollerPid.setReference(1, ControlType.kMAXMotionVelocityControl);
  }

  // @Override
  // public double getRollerCurrent() {
  //   //return intakeRoller.getCure();
  // }

  // @Override
  // public double getEncoder() {
  //   //return encoder.getPosition();
  // }

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
