package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANandPowerPorts;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX intakeRoller =
      new TalonFX(CANandPowerPorts.INTAKE_ROLLER.getDeviceNumber());

  private final TalonFX intakePivot = new TalonFX(CANandPowerPorts.INTAKE_PIVOT.getDeviceNumber());

  public final int[] powerPorts = {
    CANandPowerPorts.INTAKE_PIVOT.getPowerPort(), CANandPowerPorts.INTAKE_ROLLER.getPowerPort()
  };

  private final StatusSignal<Angle> pivotPosition = intakePivot.getPosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = intakePivot.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = intakePivot.getMotorVoltage();
  private final StatusSignal<Current> pivotCurrent = intakePivot.getSupplyCurrent();

  public IntakeIOKraken() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(pivotPosition.getValueAsDouble()) / 21.42857; // gear ratio
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(pivotVelocity.getValueAsDouble()) / 21.42857; // gear ratio
    inputs.appliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {pivotCurrent.getValueAsDouble()};
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

  @Override
  public void setPivotPosition(double position) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    intakePivot.setControl(m_request.withPosition(position));
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
