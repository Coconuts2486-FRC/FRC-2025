package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX intakeRoller = new TalonFX(21);

  private final TalonFX intakePivot = new TalonFX(22);

  public IntakeIOKraken() {}

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

  @Override
  public void configure(
      double kG,
      double kS,
      double kV,
      double kA,
      double kP,
      double kI,
      double kD,
      double velocity,
      double acceleration,
      double jerk) {

    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kG = kG;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;
    slot0Configs.kA = kA;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = velocity;
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicJerk = jerk;
  }
}
