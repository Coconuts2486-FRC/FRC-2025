package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
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

  private final CANcoder encoderActual = new CANcoder(1);

  PIDController pivotPID = new PIDController(0, 0, 0);

  public final int[] powerPorts = {
    CANandPowerPorts.INTAKE_PIVOT.getPowerPort(), CANandPowerPorts.INTAKE_ROLLER.getPowerPort()
  };

  private final StatusSignal<Angle> pivotPosition = encoderActual.getAbsolutePosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = intakePivot.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = intakePivot.getMotorVoltage();
  private final StatusSignal<Current> pivotCurrent = intakePivot.getSupplyCurrent();

  public IntakeIOKraken() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(pivotPosition.getValueAsDouble()); // 21.42857; // gear ratio
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(pivotVelocity.getValueAsDouble()); // 21.42857; // gear ratio
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
    intakePivot.set(-pivotPID.calculate(pivotPosition.getValueAsDouble(), position));
  }

  @Override
  public void rollerSpeed(double speed) {
    intakeRoller.set(speed);
  }

  @Override
  public void configure(double kP, double kI, double kD) {
    pivotPID.setP(kP);
    pivotPID.setI(kI);
    pivotPID.setD(kD);
  }

  @Override
  public double getEncoder() {
    return pivotPosition.getValueAsDouble();
  }
}
