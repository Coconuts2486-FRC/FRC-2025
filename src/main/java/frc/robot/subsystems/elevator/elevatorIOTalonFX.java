package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.CANandPowerPorts;

public class elevatorIOTalonFX implements elevatorIO {

  // motor called in
  private final TalonFX elevator =
      new TalonFX(
          CANandPowerPorts.ELEVATOR_MOTOR.getDeviceNumber(),
          CANandPowerPorts.ELEVATOR_MOTOR.getBus());

  public final int[] powerPorts = {CANandPowerPorts.ELEVATOR_MOTOR.getPowerPort()};

  public elevatorIOTalonFX() {}

  @Override
  public void setPosistion(double posistion) {
    final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    elevator.setControl(motionMagic.withPosition(posistion));
  }

  @Override
  public void stop() {
    elevator.stopMotor();
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

    elevator.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void setVoltage(double volts) {
    elevator.setControl(new VoltageOut(volts));
  }
}
