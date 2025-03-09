package frc.robot.subsystems.algae_mech;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANandPowerPorts;

public class AlgaeMechIOTalonFX implements AlgaeMechIO {
  private final TalonFX roller = new TalonFX(CANandPowerPorts.ALGAE_ROLLER.getDeviceNumber());
  private final TalonFX pivot = new TalonFX(CANandPowerPorts.ALGAE_PIVOT.getDeviceNumber());
  private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);
  private final PIDController pivotController = new PIDController(10, 0, 0);

  public AlgaeMechIOTalonFX() {

    var PIDConfig = new Slot0Configs();
    PIDConfig.kP = 1;
    PIDConfig.kI = 0;
    PIDConfig.kD = 0;

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    var ramp = new OpenLoopRampsConfigs();
    ramp.withDutyCycleOpenLoopRampPeriod(0.25);
    roller.getConfigurator().apply(rollerConfig);
    roller.getConfigurator().apply(PIDConfig);
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivot.getConfigurator().apply(pivotConfig);
  }

  // Define the leader / follower motors from the Ports section of RobotContainer

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    roller.setControl(
        new VelocityDutyCycle(Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)));
  }

  @Override
  public void setPercent(double percent) {
    roller.set(percent);
  }

  @Override
  public void stop() {
    roller.stopMotor();
    pivot.stopMotor();
  }

  @Override
  public void pivotToPosition(double position) {
    pivot.setControl(new DutyCycleOut(-pivotController.calculate(pivotEncoder.get(), position)));
  }

  @Override
  public double getCurrent() {

    return roller.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public double getEncoderPose() {
    return pivotEncoder.get();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    // leader.getConfigurator().apply(config);
  }
}
