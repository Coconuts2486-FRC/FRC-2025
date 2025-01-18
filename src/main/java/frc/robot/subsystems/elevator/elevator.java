package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.elevatorConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class elevator extends RBSISubsystem {
  private final SimpleMotorFeedforward ffModel;
  private final elevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public elevator(elevatorIO io) {
    this.io = io;

    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(kStaticGainReal, kVelocityGainReal);
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
        ffModel = new SimpleMotorFeedforward(kStaticGainSim, kVelocityGainSim);
        io.configure(
            kGSim, kSSim, kVSim, kASim, kPSim, kISim, kDSim, kVelocity, kAcceleration, kJerk);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        io.configure(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
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
