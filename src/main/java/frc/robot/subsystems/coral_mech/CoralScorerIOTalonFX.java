package frc.robot.subsystems.coral_mech;

import edu.wpi.first.wpilibj.DigitalInput;

public class CoralScorerIOTalonFX implements CoralScorerIO {

  private final DigitalInput lightStop = new DigitalInput(5);

  @Override
  public boolean getLightStop() {
    return lightStop.get();
  }
}
