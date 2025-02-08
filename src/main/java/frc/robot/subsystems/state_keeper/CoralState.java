// Copyright (c) 2025 FRC 2486
// http://github.com/Coconuts2486-FRC
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.state_keeper;

import frc.robot.util.VirtualSubsystem;

public class CoralState extends VirtualSubsystem {
  // scoreStates[0] stores horizontal state
  // scoreStates[1] stores vertical state
  // scoreStates[2] stores left or right state
  private int[] scoreStates;

  public CoralState() {
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

  public void indexDown() {

    scoreStates[1] = scoreStates[1] - 1;
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
