package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANandPowerPorts;
import java.util.function.BooleanSupplier;

public class ElevatorIOTalonFX implements ElevatorIO {

  // motor called in
  private final TalonFX elevatorMotor =
      new TalonFX(
          CANandPowerPorts.ELEVATOR_MOTOR.getDeviceNumber(),
          CANandPowerPorts.ELEVATOR_MOTOR.getBus());

  private final StatusSignal<AngularVelocity> velocity = elevatorMotor.getVelocity();

  public final int[] powerPorts = {CANandPowerPorts.ELEVATOR_MOTOR.getPowerPort()};

  private final StatusSignal<Angle> elevatorPosition = elevatorMotor.getPosition();
  private final StatusSignal<AngularVelocity> elevatorVelocity = elevatorMotor.getVelocity();
  private final StatusSignal<Voltage> elevatorAppliedVolts = elevatorMotor.getMotorVoltage();
  private final StatusSignal<Current> elevatorCurrent = elevatorMotor.getSupplyCurrent();

  public ElevatorIOTalonFX() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        elevatorPosition, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(elevatorPosition.getValueAsDouble()) / kElevatorGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(elevatorVelocity.getValueAsDouble()) / kElevatorGearRatio;
    inputs.appliedVolts = elevatorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {elevatorCurrent.getValueAsDouble()};
  }

  @Override
  public void setPosistion(double posistion) {
    final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    elevatorMotor.setControl(motionMagic.withPosition(posistion));
  }

  @Override
  public void stop() {
    elevatorMotor.stopMotor();
  }

  @Override
  public void configure(
      double Kg,
      double Ks,
      double Kv,
      double Ka,
      double Kp,
      double Ki,
      double Kd,
      double velocity,
      double aceleration,
      double jerk) {
    var talonFXConfigs = new TalonFXConfiguration();
    var talonSlot0Configs = talonFXConfigs.Slot0;
    var motionMagicConfigs = talonFXConfigs.MotionMagic;

    talonSlot0Configs.kG = Kg;
    talonSlot0Configs.kS = Ks;
    talonSlot0Configs.kV = Kv;
    talonSlot0Configs.kA = Ka;
    talonSlot0Configs.kP = Kp;
    talonSlot0Configs.kI = Ki;
    talonSlot0Configs.kD = Kd;

    motionMagicConfigs.MotionMagicCruiseVelocity = velocity;
    motionMagicConfigs.MotionMagicAcceleration = aceleration;
    motionMagicConfigs.MotionMagicJerk = jerk;

    elevatorMotor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void limit(BooleanSupplier limitSwitch) {
    if (limitSwitch.getAsBoolean()) {}
  }

  @Override
  public void setCoast() {
    elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void setBrake() {
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }
}
