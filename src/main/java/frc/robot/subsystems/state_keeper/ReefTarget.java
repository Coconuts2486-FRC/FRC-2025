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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

/** This class stores information about the targeted scoring location on the REEF */
public class ReefTarget extends VirtualSubsystem {
  private int reefPostAll = 1;
  private int reefPostLR = 0;
  private int reefLevel = 1;
  public double elevatorDelay = 0;

  private static ReefTarget instance;

  public static ReefTarget getInstance() {
    if (instance == null) {
      instance = new ReefTarget();
    }
    return instance;
  }

  /** Constructor */
  private ReefTarget() {}

  /** Periodic function includes logging and publishing to NT */
  public synchronized void periodic() {
    getElevatorDelay();
    // Log to AdvantageKit
    Logger.recordOutput("ReefTarget/Post_ALL", convertIntToAlphabet(reefPostAll));
    Logger.recordOutput("ReefTarget/Post_LR", reefPostLR);
    Logger.recordOutput("ReefTarget/Level", reefLevel);
    Logger.recordOutput("ReefTarget/ElevatorHeight", getElevatorHeight());
  }

  /** Index the desired scoring state up one */
  public void indexUp() {
    reefLevel = Math.min(++reefLevel, 4);
    System.out.println(reefLevel);
  }

  /** Index the desired scoring state down one */
  public void indexDown() {
    reefLevel = Math.max(--reefLevel, 1);
    System.out.println(reefLevel);
  }

  /**
   * Index the desired scoring state right one
   *
   * <p>Following standard number line conventions, right is a larger integer.
   */
  public void indexRight() {
    reefPostLR = Math.min(++reefPostLR, 1);
    // Continuously wrap the A-L designation
    if (++reefPostAll > 12) {
      reefPostAll -= 12;
    }
  }

  /**
   * Index the desired scoring state left one
   *
   * <p>Following standard number line conventions, left is a smaller integer.
   */
  public void indexLeft() {
    reefPostLR = Math.max(--reefPostLR, 0);
    // Continuously wrap the A-L designation
    if (--reefPostAll < 1) {
      reefPostAll += 12;
    }
  }

  /** Utility function for converting integer to letter */
  public static String convertIntToAlphabet(int num) {
    if (num < 1 || num > 26) {
      throw new IllegalArgumentException("Number must be between 1 and 26");
    }
    return String.valueOf((char) ('A' + num - 1));
  }

  /* Setter functions to set kept state from Auto Commands ***************** */
  /**
   * Set the reef level state keeper
   *
   * <p>This method should only be called from Autonomous Commands
   *
   * @param level The reef level (1-4) to which the robot will score
   */
  public void setReefLevel(int level) {
    if (level >= 1 && level <= 4) {
      reefLevel = level;
    } else {
      // Raise Alert that improper level was set
    }
  }

  /**
   * Set the reef post Alphabetic state keeper
   *
   * <p>This method should only be called from Autonomous Commands
   *
   * @param post The reef post (0-11) to which the robot will score
   */
  public void setReefPost(int post) {
    if (post >= 0 && post <= 11) {
      reefPostAll = post;
      reefPostLR = post % 2; // A is the left
    } else {
      // Raise Alert that improper post was set
    }
  }

  /* Getter functions to return kept state ********************************* */
  /** Return the elevator height needed to score the coral at the stored level */
  public Distance getElevatorHeight() {
    switch (reefLevel) {
      case 2:
        return ElevatorConstants.kL2;
      case 3:
        return ElevatorConstants.kL3;
      case 4:
        return ElevatorConstants.kL4;
      default:
        return ElevatorConstants.kL2;
    }
  }

  public void getElevatorDelay() {

    if (reefLevel == 2) {
      elevatorDelay = .35;
    } else if (reefLevel == 3) {
      elevatorDelay = .55;
    } else if (reefLevel == 4) {
      elevatorDelay = .85;
    } else {
      elevatorDelay = .35;
    }
  }

  /** Return the L-R pose needed to line up with the reef post */
  public Pose2d getReefPose(Pose2d currentPose) {

    // TODO: Use the current pose plus L-R information to determine which Pose2d() to return

    return new Pose2d();
  }

  /** Return the A-L pose needed to line up with the reef post */
  public Pose2d getReefPose() {
    // TODO: Put all of the reef poses here, based on alliance
    switch (reefPostAll) {
      case 0:
        // Post A
        return new Pose2d();
      case 1:
        // Post B
        return new Pose2d();
      case 2:
        // Post C
        return new Pose2d();
      case 3:
        // Post D
        return new Pose2d();
      case 4:
        // Post E
        return new Pose2d();
      case 5:
        // Post F
        return new Pose2d();
      case 6:
        // Post G
        return new Pose2d();
      case 7:
        // Post H
        return new Pose2d();
      case 8:
        // Post I
        return new Pose2d();
      case 9:
        // Post J
        return new Pose2d();
      case 10:
        // Post K
        return new Pose2d();
      case 11:
        // Post L
        return new Pose2d();
      default:
        // Shouldn't run, but required case
        return new Pose2d();
    }
  }

  /** Return the integer value of the reef level */
  public int getReefLevel() {
    return reefLevel;
  }

  /** Return the integer value of the LR reef post */
  public int getReefPostLR() {
    return reefPostLR;
  }

  /** Return the integer value of the reef post */
  public int getReefPostAll() {
    return reefPostAll;
  }
}
