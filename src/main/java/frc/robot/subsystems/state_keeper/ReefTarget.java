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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.DriveToPositionConstatnts;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * This class stores information about the targeted scoring location on the REEF
 *
 * <p>Reef level is encoded as an INT between 1 and 4 corresponding to the Tough, L2, L3, and L4
 * posts, respectively.
 *
 * <p>Reef Post going around (A-L) is encoded as a zero-indexed INT between 0 and 11.
 *
 * <p>Reef Post LR is encoded as a 0 (left) or 1 (right).
 */
public class ReefTarget extends VirtualSubsystem {
  private int reefPostAll = 0;
  private int reefPostLR = 0;
  private int reefLevel = 1;
  public double elevatorDelay = 0;

  private static ReefTarget instance;

  /** Return an instance of this class */
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
    Logger.recordOutput("ReefTarget/Post_ALL", convertIntToAlphabet(reefPostAll + 1));
    Logger.recordOutput("ReefTarget/Post_LR", reefPostLR);
    Logger.recordOutput("ReefTarget/Level", reefLevel);
    Logger.recordOutput("ReefTarget/ElevatorHeight", getElevatorHeight());
    Logger.recordOutput("ReefTarget/ScoringPose", getReefPose());
  }

  /** Index the desired scoring state up one */
  public void indexUp() {
    reefLevel = Math.min(++reefLevel, 4);
  }

  /** Index the desired scoring state down one */
  public void indexDown() {
    reefLevel = Math.max(--reefLevel, 1);
  }

  /**
   * Index the desired scoring state right one
   *
   * <p>Following standard number line conventions, right is a larger integer.
   */
  public void indexRight() {
    // Continuously wrap the A-L designation
    if (++reefPostAll >= 12) {
      reefPostAll -= 12;
    }
    reefPostLR = reefPostAll % 2;
  }

  /**
   * Index the desired scoring state left one
   *
   * <p>Following standard number line conventions, left is a smaller integer.
   */
  public void indexLeft() {
    // Continuously wrap the A-L designation
    if (--reefPostAll < 0) {
      reefPostAll += 12;
    }
    reefPostLR = reefPostAll % 2;
  }

  /**
   * Utility function for converting integer to letter
   *
   * @param num The integer (1-26) to convert to a letter
   */
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
        return ElevatorConstants.kElevatorZeroHeight;
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
    switch (reefPostAll) {
      case 0:
        // Post A
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 7;
              case Blue -> 18;
            },
            ScoringPosition.LEFT);

      case 1:
        // Post B
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 7;
              case Blue -> 18;
            },
            ScoringPosition.RIGHT);

      case 2:
        // Post C
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 8;
              case Blue -> 17;
            },
            ScoringPosition.LEFT);

      case 3:
        // Post D
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 8;
              case Blue -> 17;
            },
            ScoringPosition.RIGHT);

      case 4:
        // Post E
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 9;
              case Blue -> 22;
            },
            ScoringPosition.LEFT);

      case 5:
        // Post F
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 9;
              case Blue -> 22;
            },
            ScoringPosition.RIGHT);

      case 6:
        // Post G
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 10;
              case Blue -> 21;
            },
            ScoringPosition.LEFT);

      case 7:
        // Post H
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 10;
              case Blue -> 21;
            },
            ScoringPosition.RIGHT);

      case 8:
        // Post I
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 11;
              case Blue -> 20;
            },
            ScoringPosition.LEFT);

      case 9:
        // Post J
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 11;
              case Blue -> 20;
            },
            ScoringPosition.RIGHT);

      case 10:
        // Post K
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 6;
              case Blue -> 19;
            },
            ScoringPosition.LEFT);

      case 11:
        // Post L
        return computeReefPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 6;
              case Blue -> 19;
            },
            ScoringPosition.RIGHT);

      default:
        // Shouldn't run, but required case and useful for testing
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

  /* Utility Functions ***************************************************** */
  /** The scoring position at any one REEF face */
  private enum ScoringPosition {
    LEFT, // Left reef post (coral)
    RIGHT, // Right reef post (coral)
    CENTER // Center position (algae grab)
  }

  /**
   * Compute the pose of the robot to score at the specified REEF station
   *
   * <p>This method takes the AprilTag pose from the field and applies a translation with respect to
   * the tag to find the desired robot position.
   *
   * @param tagID The AprilTag ID of the REEF face at which we are to score
   * @param position The `ScoringPosition` at which to place the robot (left post, right post, or
   *     centered for algae grab)
   */
  private Pose2d computeReefPose(int tagID, ScoringPosition position) {
    Optional<Pose3d> tagPose = AprilTagConstants.aprilTagLayout.getTagPose(tagID);

    // For whatever reason, if the specified tag pose doesn't exist, return empty pose
    if (tagPose.isEmpty()) {
      return new Pose2d();
    }

    // Return the transformed location
    return tagPose
        .get()
        .toPose2d()
        .transformBy(
            new Transform2d(
                switch (position) {
                  case LEFT -> DriveToPositionConstatnts.kLeftReefPost;
                  case RIGHT -> DriveToPositionConstatnts.kRightReefPost;
                  case CENTER -> DriveToPositionConstatnts.kAlgaeGrab;
                },
                new Rotation2d()));
  }
}
