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
  // Define Hardware and controllers
  private final TalonFX m_roller = new TalonFX(CANandPowerPorts.ALGAE_ROLLER.getDeviceNumber());
  private final TalonFX m_pivot = new TalonFX(CANandPowerPorts.ALGAE_PIVOT.getDeviceNumber());
  private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private final DutyCycleEncoder pivotEncoder =
      new DutyCycleEncoder(CANandPowerPorts.ALGAE_PIVOT_ENCODER);
  private final PIDController pivotController = new PIDController(10, 0, 0);

  // * Constructor */
  public AlgaeMechIOTalonFX() {

    var PIDConfig = new Slot0Configs();
    PIDConfig.kP = 1;
    PIDConfig.kI = 0;
    PIDConfig.kD = 0;

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    var ramp = new OpenLoopRampsConfigs();
    ramp.withDutyCycleOpenLoopRampPeriod(0.25);
    m_roller.getConfigurator().apply(rollerConfig);
    m_roller.getConfigurator().apply(PIDConfig);
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_pivot.getConfigurator().apply(pivotConfig);
  }

  @Override
  public void setPivotVoltage(double volts) {}

  @Override
  public void setRollerVoltage(double volts) {}

  @Override
  public void setRollerVelocity(double velocityRadPerSec, double ffVolts) {
    m_roller.setControl(
        new VelocityDutyCycle(Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)));
  }

  @Override
  public void setRollerPercent(double percent) {
    m_roller.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void stop() {
    m_roller.stopMotor();
    m_pivot.stopMotor();
  }

  @Override
  public void pivotToPosition(double position) {
    m_pivot.setControl(new DutyCycleOut(-pivotController.calculate(pivotEncoder.get(), position)));
  }

  @Override
  public double getRollerCurrent() {
    return m_roller.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public double getPivotEncoderPose() {
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

  /** Return the list of PDH power ports used for this mechanism */
  @Override
  public int[] getPowerPorts() {
    return powerPorts;
  }
}
