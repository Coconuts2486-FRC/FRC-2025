package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends RBSISubsystem {
  private final ElevatorFeedforward ffModel;
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Elevator(ElevatorIO io) {
    this.io = io;

    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        ffModel = new ElevatorFeedforward(kSReal, kGReal, kVReal, kAReal);
        io.configure(
            kGReal,
            kSReal,
            kVReal,
            kAReal,
            kPReal,
            kIReal,
            kDReal,
            kVelocity,
            kAcceleration,
            kJerk);
        break;
      case SIM:
        ffModel = new ElevatorFeedforward(kSSim, kGSim, kVSim, kASim);
        io.configure(
            kGSim, kSSim, kVSim, kASim, kPSim, kISim, kDSim, kVelocity, kAcceleration, kJerk);
        break;
      default:
        ffModel = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
        io.configure(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).div(Seconds.of(1.5)), // QuasiStatis
                Volts.of(1.5), // Dynamic
                Seconds.of(2.0),
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Units.Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setPosistion(double posistion) {
    io.setPosistion(posistion);
  }

  public void stop() {
    io.stop();
  }

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
    io.configure(Kg, Ks, Kv, Ka, Kp, Ki, Kd, velocity, aceleration, jerk);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void setCoast() {
    io.setCoast();
  }

  public void setBrake() {
    io.setBrake();
  }
}
