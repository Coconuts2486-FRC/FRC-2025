package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class intakeIOTalonFX implements intakeIO {
  TalonFX intakeTalonFX = new TalonFX(999);
}
