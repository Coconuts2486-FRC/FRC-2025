package frc.robot.subsystems.state_keeper;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.DriveToPositionConstatnts;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public class OtherTargets {
  private int station = 0;
  private int playerStation = 0;
  private Drive m_drive;

  private static OtherTargets instance;

  /** Return an instance of this class */
  public static OtherTargets getInstance(Drive drive) {
    if (instance == null) {
      instance = new OtherTargets(drive);
    }
    return instance;
  }

  /** Constructor */
  private OtherTargets(Drive drive) {
    this.m_drive = drive;
  }

  @AutoLogOutput(key = "ReefTarget/ReefFacePose")
  public Pose2d getClosestStationPose() {

    Pose2d thisPose = m_drive.getPose();

    int baseTag =
        switch (DriverStation.getAlliance().get()) {
          case Red -> 1;
          case Blue -> 12;
        };
    double mindist = 100.0; // meters
    int wantedTag = 0;

    for (int i = 0; i < 2; i++) {
      Transform2d relPose =
          thisPose.minus(AprilTagConstants.aprilTagLayout.getTagPose(baseTag + i).get().toPose2d());
      double absDist =
          Math.sqrt(
              Math.pow(relPose.getMeasureX().in(Meters), 2.0)
                  + Math.pow(relPose.getMeasureY().in(Meters), 2.0));
      if (absDist < mindist) {
        wantedTag = baseTag + i;
        mindist = Math.min(absDist, mindist);
      }
    }

    // Determine which A-L post we are going for based on out `wantedTag

    // Return the pose computed from the wanted tag and the LR position
    return computePlayerStationPose(wantedTag);
  }

  private Pose2d computePlayerStationPose(int tagID) {
    Optional<Pose3d> tagPose = AprilTagConstants.aprilTagLayout.getTagPose(tagID);

    // For whatever reason, if the specified tag pose doesn't exist, return empty pose
    if (tagPose.isEmpty()) {
      return new Pose2d();
    }

    // Return the transformed location
    return tagPose
        .get()
        .toPose2d()
        .transformBy(new Transform2d(DriveToPositionConstatnts.kStation, new Rotation2d()));
  }

  public Pose2d getPlayerStationPose() {
    switch (playerStation) {
      case 0:
        // Post A
        return computePlayerStationPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 1;
              case Blue -> 12;
            });

      case 1:
        // Post B
        return computePlayerStationPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 2;
              case Blue -> 13;
            });

      default:
        return computePlayerStationPose(
            switch (DriverStation.getAlliance().get()) {
              case Red -> 1;
              case Blue -> 12;
            });
    }
  }
}
