package frc.robot.subsystems.Controls;

import frc.robot.util.VirtualSubsystem;

public class CoralControl extends VirtualSubsystem {
  // scoreStates[0] stores horizontal state
  // scoreStates[1] stores vertical state
  // scoreStates[2] stores left or right state
  private int[] scoreStates;

  public CoralControl() {
    scoreStates[0] = 0;
    scoreStates[1] = 0;
    scoreStates[2] = 0;
  }

  public void indexHorizontal(boolean indexRight, boolean indexLeft) {
    if (indexRight) {
      scoreStates[0] = scoreStates[0] + 1;
    }
    if (indexLeft) {
      scoreStates[0] = scoreStates[0] - 1;
    }

    if (scoreStates[0] < 0) {
      scoreStates[0] = 11;
    }
    if (scoreStates[0] > 11) {
      scoreStates[0] = 0;
    }
  }

  public void indexUp(boolean indexUp, boolean indexDown) {

      scoreStates[1] = scoreStates[1] + 1;
      if (scoreStates[1] > 3) {
        scoreStates[1] = 3;
      }
  }
  public void indexDown(){
   
      scoreStates[1] =  scoreStates[1] - 1;
    if (scoreStates[1] < 0) {
      scoreStates[1] = 0;
    }
  }

  public void indexR() {
      scoreStates[2] = 1;

  }
  public void indexL() {
    scoreStates[2] = 0;
  }

  @Override
  public void periodic() {}

  public int[] getScoreState() {
    return scoreStates;
  }
}
