package frc.robot.subsystems.CoralScorer;

import frc.robot.util.RBSISubsystem;

public class CoralScorer extends RBSISubsystem {
  private final CoralScorerIO io;


  public CoralScorer(CoralScorerIO io) {
    this.io = io;
  }


  public void runVolts(double volts){
    io.setVolts(volts);
  }

  public void stop(){
    io.stop();
  }
  


}
