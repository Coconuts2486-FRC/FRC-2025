package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.intake.intakeIO;

public class intakeIOTalonFX implements intakeIO{
  private final TalonFX intakeTalonFX = new TalonFX(deviceId:999, MotorType.kbrushless);

}
