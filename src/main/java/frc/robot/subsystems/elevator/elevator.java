package frc.robot.subsystems.elevator;

import static frc.robot.Constants.elevatorConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;

public class elevator extends RBSISubsystem {
  private final SimpleMotorFeedforward ffModel;
  private final elevatorIO io;

  public elevator(elevatorIO io) {
    this.io = io;

    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(kStaticGainReal, kVelocityGainReal);
        io.configure(
            kgreal, ksreal, kvreal, kareal, kpreal, kireal, kdreal, velocity, acceleration, jerk);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(kStaticGainSim, kVelocityGainSim);
        io.configure(kgSim, ksSim, kvSim, kaSim, kpSim, kiSim, kdSim, velocity, acceleration, jerk);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        io.configure(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        break;
    }
  }

  public void setPosistion(double posistion) {
    io.setPosistion(posistion);
  }

  public void stop() {
    io.stop();
  }

  public void confiure(
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
}
