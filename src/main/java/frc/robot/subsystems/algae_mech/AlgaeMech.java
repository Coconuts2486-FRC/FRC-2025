package frc.robot.subsystems.algae_mech;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

// hi :D
public class AlgaeMech extends RBSISubsystem {
  private final AlgaeMechIO io;
  // private final AlgaeMechIOInputsAutoLogged inputs = new AlgaeMechIOInputsAutoLogged();
  // private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  public AlgaeMech(AlgaeMechIO io) {
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
                (state) -> Logger.recordOutput("AlgaeMech/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("Flywheel", inputs);
  }

  public void printEncoder() {
    System.out.println(io.getEncoderPose());
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  // TODO fix ffvolts
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    // io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
    io.setVelocity(velocityRPM, 0);
    // Log flywheel setpoint
    Logger.recordOutput("algae_mech/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  public void pivotToPosition(double position) {
    io.pivotToPosition(position);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  // Pibot to stored position
  public void pivotUp() {
    io.pivotToPosition(.209);
  }

  // Pivots to position to pick up off floor
  public void pivotHorizontal() {
    // io.pivotToPosition(.45);
    io.pivotToPosition(.45);
  }

  // Pivots to remove coral from reef
  public void pivotOffReef() {
    io.pivotToPosition(.521);
  }
  // /** Returns the current velocity in RPM. */
  // @AutoLogOutput(key = "Mechanism/Flywheel")
  // //public double getVelocityRPM() {
  //   //  return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  // }
}
