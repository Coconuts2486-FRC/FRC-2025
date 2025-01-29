package frc.robot.subsystems.Intake;

import edu.wpi.first.units.Units;
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
                Volts.of(1.0).div(Seconds.of(1.5)), // QuasiStatis
                Volts.of(1.5), // Dynamic
                Seconds.of(2.0),
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Units.Volts)), null, this));
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
