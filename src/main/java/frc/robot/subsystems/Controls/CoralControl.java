package frc.robot.subsystems.Controls;

import frc.robot.util.VirtualSubsystem;

public class CoralControl extends VirtualSubsystem{
  // scoreStates[0] = horizontal states
  // scoreStates[1] = vertical states
  private int[] scoreStates;

  public CoralControl() {
    scoreStates[0] = 0;
    scoreStates[1] = 0;
  }

  public void indexHorizontal(boolean indexUp, boolean indexDown) {
    if (indexUp) {
      scoreStates[0] = scoreStates[0] + 1;
    }
    if (indexDown) {
      scoreStates[0] = scoreStates[0] - 1;
    }

    if(scoreStates[0] < 0){
      scoreStates[0] = 11;
    }
    if(scoreStates[0] > 11){
      scoreStates[0] = 0;
    }
  }

  public void indexVertical(boolean indexUp, boolean indexDown) {
    if (indexUp) {
      scoreStates[1] = scoreStates[1] + 1;
    }
    if (indexDown) {
      scoreStates[1] = scoreStates[1] - 1;
    }

    if(scoreStates[1] > 3){
      scoreStates[1] = 3;
    }
    if(scoreStates[1] < 0){
      scoreStates[1] = 0;
    }

  }

  @Override
  public void periodic() {}

  public int[] getScoreState(){
    return scoreStates;
  }

  
}
