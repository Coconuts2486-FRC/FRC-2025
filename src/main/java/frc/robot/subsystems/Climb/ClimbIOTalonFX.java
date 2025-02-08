package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.CANandPowerPorts;

public class ClimbIOTalonFX implements ClimbIO {
  private final Servo climbExtender = new Servo(CANandPowerPorts.CLIMB_SERVO);
  private final TalonFX climber = new TalonFX(CANandPowerPorts.CLIMB.getDeviceNumber());

  public ClimbIOTalonFX() {
    var climbConfig = new TalonFXConfiguration();
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  }

  @Override
  public void turnClimbServo(double position) {
    climbExtender.set(position);
  }

  @Override
  public void twistMotorToPosition(double position) {
    climber.setControl(new PositionDutyCycle(position));
  }

  @Override
  public void twistMotorVolts(double volts) {
    climber.setVoltage(volts);
  }

  @Override
  public void configPID(double kP, double kI, double kD) {
    Slot0Configs pid = new Slot0Configs();
    pid.withKP(kP);
    pid.withKI(kI);
    pid.withKD(kD);
    climber.getConfigurator().apply(pid);
  }
}
