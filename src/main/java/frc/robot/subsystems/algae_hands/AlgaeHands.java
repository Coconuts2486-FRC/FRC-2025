package frc.robot.subsystems.algae_hands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class AlgaeHands extends RBSISubsystem {
  private final AlgaeHandsIO io;
  // private final AlgaeHandsIOInputsAutoLogged inputs = new AlgaeHandsIOInputsAutoLogged();
  // private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  public AlgaeHands(AlgaeHandsIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    // switch (Constants.getMode()) {
    //   case REAL:
    //   case REPLAY:
    // ffModel = new SimpleMotorFeedforward(kStaticGainReal, kVelocityGainReal);
    //     io.configurePID(pidReal.kP, pidReal.kI, pidReal.kD);
    //     break;
    //   case SIM:
    //     ffModel = new SimpleMotorFeedforward(kStaticGainSim, kVelocityGainSim);
    //     io.configurePID(pidSim.kP, pidSim.kI, pidSim.kD);
    //     break;
    //   default:
    //     ffModel = new SimpleMotorFeedforward(0.0, 0.0);
    //     break;
    // }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("AlgaeHands/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("Flywheel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  //TODO fix ffvolts
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    // io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
    io.setVelocity(velocityRPM, 0);
    // Log flywheel setpoint
    Logger.recordOutput("algae_hands/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  // /** Returns the current velocity in RPM. */
  // @AutoLogOutput(key = "Mechanism/Flywheel")
  // //public double getVelocityRPM() {
  //   //  return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  // }
}
