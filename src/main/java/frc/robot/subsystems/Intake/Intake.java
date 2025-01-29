package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.RBSISubsystem;

public class Intake extends RBSISubsystem {
  private final IntakeIO io;
  private final SysIdRoutine sysId;

  public Intake(IntakeIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  public void setPivotPosition(double position) {
    io.setPivotPosition(position);
  }

  public void runPivotVolts(double volts) {
    io.setPivotVolts(volts);
  }

  public void setRollerVolts(double volts) {
    io.setRollerVolts(volts);
  }

  public void stop() {
    io.stop();
  }

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
    io.configure(kG, kS, kV, kA, kP, kI, kD, velocity, acceleration, jerk);
  }

  @Override
  public int[] getPowerPorts() {
    return io.powerPorts;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
