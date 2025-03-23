// Copyright (c) 2025 FRC 2486
// http://github.com/Coconuts2486-FRC
// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
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
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.AprilTagConstants.AprilTagLayoutType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.RBSIEnum.AutoType;
import frc.robot.util.RBSIEnum.CTREPro;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIEnum.MotorIdleMode;
import frc.robot.util.RBSIEnum.SwerveType;
import frc.robot.util.RBSIEnum.VisionType;
import frc.robot.util.RobotDeviceId;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /***************************************************************************/
  /**
   * Define the various multiple robots that use this same code (e.g., LEONARDO, DEVBOT, SIMBOT,
   * etc.) and the operating modes of the code (REAL, SIM, or REPLAY)
   */
  private static RobotType robotType = RobotType.LEONARDO;

  // Define swerve, auto, and vision types being used
  // NOTE: Only PHOENIX6 swerve base has been tested at this point!!!
  //       If you have a swerve base with non-CTRE compoments, use YAGSL
  //       under strict caveat emptor -- and submit any error and bugfixes
  //       via GitHub issues.
  private static SwerveType swerveType = SwerveType.PHOENIX6; // PHOENIX6, YAGSL
  private static CTREPro phoenixPro = CTREPro.LICENSED; // LICENSED, UNLICENSED
  private static AutoType autoType = AutoType.PATHPLANNER; // PATHPLANNER, CHOREO
  private static VisionType visionType = VisionType.PHOTON; // PHOTON, LIMELIGHT, NONE

  /** Enumerate the robot types (name your robots here) */
  public static enum RobotType {
    GEORGE, // Development / Alpha / Practice Bot, a.k.a. "George"
    LEONARDO, // Competition robot, a.k.a. "Leonardo de Pinchy"
    SIMBOT // Simulated robot
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType);
      System.exit(1);
    }
  }

  /** Disable the Hardware Abstraction Layer, if requested */
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /***************************************************************************/
  /* The remainder of this file contains physical and/or software constants for the various subsystems of the robot */

  /** General Constants **************************************************** */
  public static final double loopPeriodSecs = 0.02;

  public static final boolean tuningMode = true;

  /** Physical Constants for Robot Operation ******************************* */
  public static final class PhysicalConstants {

    public static final double kRobotMassKg = Units.lbsToKilograms(100.);
    public static final Matter kChassis =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMassKg);
    // Robot moment of intertial; this can be obtained from a CAD model of your drivetrain. Usually,
    // this is between 3 and 8 kg*m^2.
    public static final double kRobotMOI = 6.94;

    // Wheel coefficient of friction
    public static final double kWheelCOF = 1.2;

    // Measured Wheel Radius
    public static final Distance kWheelRadius = Inches.of(2.033);
  }

  /** Power Distribution Constants ********************************** */
  public static final class PowerConstants {

    // Current Limits
    public static final double kTotalMaxCurrent = 120.;
    public static final double kMotorPortMaxCurrent = 40.;
    public static final double kSmallPortMaxCurrent = 20.;
  }

  /** Drive Base Constants ************************************************* */
  public static final class DrivebaseConstants {

    // Theoretical free speed (m/s) at 12v applied output;
    // IMPORTANT: Follow the AdvantageKit instructions for measuring the ACTUAL maximum linear speed
    // of YOUR ROBOT, and replace the estimate here with your measured value!
    public static final double kMaxLinearSpeed = Units.feetToMeters(18);

    // Set 3/4 of a rotation per second as the max angular velocity (radians/sec)
    public static final double kMaxAngularSpeed = 1.5 * Math.PI;

    // Maximum chassis accelerations desired for robot motion  -- metric / radians
    // TODO: Compute the maximum linear acceleration given the PHYSICS of the ROBOT!
    public static final double kMaxLinearAccel = 4.0; // m/s/s
    public static final double kMaxAngularAccel = Units.degreesToRadians(720);

    // Hold time on motor brakes when disabled
    public static final double kWheelLockTime = 10; // seconds

    // SysID characterization constants
    public static final double kMaxV = 12.0; // Max volts
    public static final double kDelay = 3.0; // seconds
    public static final double kQuasiTimeout = 5.0; // seconds
    public static final double kDynamicTimeout = 3.0; // seconds

    // Default TalonFX Gains (Replaces what's in Phoenix X's Tuner Constants)
    // NOTE: Default values from 6328's 2025 Public Code
    public static final double kDriveP = 40.0;
    public static final double kDriveD = 0.03;
    public static final double kDriveV = 0.83;
    public static final double kDriveS = 0.21;
    public static final double kDriveT =
        SwerveConstants.kDriveGearRatio / DCMotor.getKrakenX60Foc(1).KtNMPerAmp;
    public static final double kSteerP = 400.0;
    public static final double kSteerD = 20.0;
  }

  /** Elevator Subsystem Constants ***************************************** */
  public static final class ElevatorConstants {

    // Idle Mode
    public static final MotorIdleMode kElevatorIdle = MotorIdleMode.BRAKE; // BRAKE, COAST

    // Physical Things on the Robot
    public static final double kElevatorGearRatio = (60.0 / 20.0) * (40.0 / 14.0);
    public static final Distance kElevatorSproketRadius = Inches.of(1.75 / 2.0);
    // These are heights off the ground of the top row of bolts on the elevator 1st stage
    public static final Distance kElevatorZeroHeight = Inches.of(40.375);
    public static final Distance kL2 = Inches.of(48.125); // Maybe 48.5
    public static final Distance kL3 = Inches.of(56);
    public static final Distance kL4 = Inches.of(68);
    public static final Distance KAlgaeLower = Inches.of(56.8);
    public static final Distance KAlgaeUpper = Inches.of(65);
    public static final Distance KAlgaeShoot = Inches.of(60);
    // Motion Magic constants
    public static final LinearVelocity kVelocity = MetersPerSecond.of(2);
    public static final LinearAcceleration kAcceleration = MetersPerSecondPerSecond.of(3);
    public static final double kJerk = 0;

    // mode real/replay
    // motor configs
    public static final double kGReal = 0.65;
    public static final double kSReal = 0.075;
    public static final double kVReal = 0.18629;
    public static final double kAReal = 0.01; // 0.000070378;
    // ka kv values found from putting elevator at a perfect 90 degree and running sys id
    public static final double kPReal = 18.0;
    public static final double kIReal = 0.0;
    public static final double kDReal = 0.01;

    // mode sim
    // motor configs
    public static final double kGSim = 0;
    public static final double kSSim = 0.1;
    public static final double kVSim = 0.05;
    public static final double kASim = 0;
    public static final double kPSim = 0;
    public static final double kISim = 0;
    public static final double kDSim = 0;
  }

  /** Coral Mechanism Subsystem Constants ********************************** */
  public static final class CoralMechConstants {

    // Idle Mode
    public static final MotorIdleMode kCoralIdle = MotorIdleMode.BRAKE; // BRAKE, COAST

    // Gear Ratio
    // TODO: Get the REAL gear ratio
    public static final double kCoralGearRatio = 10.0;

    // mode real/replay
    public static final double kStaticGainReal = 0.1;
    public static final double kVelocityGainReal = 0.05;
    // motor configs
    public static final double kSReal = 0.075;
    public static final double kVReal = 0.0018629;
    public static final double kAReal = 0;
    public static final double kPReal = 1.0;
    public static final double kIReal = 0.0;
    public static final double kDReal = 0.0;

    // mode sim
    public static final double kStaticGainSim = 0.1;
    public static final double kVelocityGainSim = 0.05;
    // motor configs
    public static final double kGSim = 0;
    public static final double kSSim = 0;
    public static final double kVSim = 0;
    public static final double kASim = 0;
    public static final double kPSim = 0;
    public static final double kISim = 0;
    public static final double kDSim = 0;
  }

  /** Intake Subsystem Constants ******************************************* */
  public static final class IntakeConstants {

    // Idle Mode
    public static final MotorIdleMode kIntakePivotIdle = MotorIdleMode.BRAKE; // BRAKE, COAST
    public static final MotorIdleMode kIntakeRollerIdle = MotorIdleMode.BRAKE; // BRAKE, COAST

    // Gear Ratio
    public static final double kIntakePivotGearRatio = 10.0;
    public static final double kIntakeRollerGearRatio = 10.0;

    // mode real/replay
    public static final double kStaticGainReal = 0.1;
    public static final double kVelocityGainReal = 0.05;
    // motor configs
    public static final double kSReal = 0.075;
    public static final double kVReal = 0.0018629;
    public static final double kAReal = 0.0;
    public static final double kPReal = 1.0;
    public static final double kIReal = 0.0;
    public static final double kDReal = 0.0;

    // mode sim
    public static final double kStaticGainSim = 0.1;
    public static final double kVelocityGainSim = 0.05;
    // motor configs
    public static final double kGSim = 0;
    public static final double kSSim = 0;
    public static final double kVSim = 0;
    public static final double kASim = 0;
    public static final double kPSim = 0;
    public static final double kISim = 0;
    public static final double kDSim = 0;

    // Motion Magic constants
    public static final double kVelocity = 1.4;
    public static final double kAcceleration = 2.8;
    public static final double kJerk = 0;
  }

  /** Algae Mechanism Subsystem Constants ********************************** */
  public static final class AlgaeMechConstants {

    // Idle Mode
    public static final MotorIdleMode kAlgaePivotIdle = MotorIdleMode.BRAKE; // BRAKE, COAST
    public static final MotorIdleMode kAlgaeRollerIdle = MotorIdleMode.BRAKE; // BRAKE, COAST

    // Gear Ratio
    // TODO: Get ACTUAL GEAR RATIOS from Eugene
    public static final double kAlgaePivotGearRatio = 10.0;
    public static final double kAlgaeRollerGearRatio = 10.0;

    // Pivot Gains
    public static final double kPPivot = 10.0;
    public static final double kIPivot = 0.0;
    public static final double kDPivot = 0.0;

    // Roller Gains
    public static final double kPRoller = 1.0;
    public static final double kIRoller = 0.0;
    public static final double kDRoller = 0.0;

    // Algae Mech Positions
    // TODO: Put these in ACTUAL DEGREES with an offset
    public static final double kStowPos = .209;
    public static final double kHorizPos = .35;
    public static final double kOtherPos = .45;
    public static final double kReefPos = .521;
    // public static final Angle kStowPos = Degrees.of(90.0);
    // public static final Angle kHorizPos = Degrees.of(0.0);
    // public static final Angle kOtherPos = Degrees.of(-10.0);
    // public static final Angle kReefPos = Degrees.of(-35.0);
    // public static final Angle kPivotOffset = Rotations.of(0.43);

  }

  /** Climb Subsystem Constants ******************************************** */
  public static final class ClimbConstants {

    // Idle Mode
    public static final MotorIdleMode kClimbIdle = MotorIdleMode.BRAKE; // BRAKE, COAST

    // Gear Ratio
    public static final double kClimbGearRatio = 10.0;

    // mode real/replay
    public static final double kStaticGainReal = 0.1;
    public static final double kVelocityGainReal = 0.05;
    // motor configs
    public static final double kSReal = 0.075;
    public static final double kVReal = 0.0018629;
    public static final double kAReal = 0.0;
    public static final double kPReal = 4.0;
    public static final double kIReal = 0.0;
    public static final double kDReal = 0.5;

    // mode sim
    public static final double kStaticGainSim = 0.1;
    public static final double kVelocityGainSim = 0.05;
    // motor configs
    public static final double kGSim = 0;
    public static final double kSSim = 0;
    public static final double kVSim = 0;
    public static final double kASim = 0;
    public static final double kPSim = 0;
    public static final double kISim = 0;
    public static final double kDSim = 0;

    // Motion Magic constants
    public static final AngularVelocity kVelocity = RotationsPerSecond.of(1.4);
    public static final AngularAcceleration kAcceleration = RotationsPerSecondPerSecond.of(2.8);

    // Put these in terms of ACTUAL PROTRACTOR ANGLES ON THE ROBOT AND A ZERO OFFSET!!!
    public static final double startClimb = .46;
    public static final double stowClimb = .72;
    public static final double completeClimb = .82;
    // public static final Angle startClimb = Degrees.of(90.0);
    // public static final Angle stowClimb = Degrees.of(180.0);
    // public static final Angle completeClimb = Degrees.of(270.0);
    // public static final Angle kClimbOffset = Rotations.of(0.50);
  }

  /** Accelerometer Constants ********************************************** */
  public static class AccelerometerConstants {

    // Insert here the orientation (CCW == +) of the Rio and IMU from the robot
    // An angle of "0." means the x-y-z markings on the device match the robot's intrinsic reference
    //   frame.
    // NOTE: It is assumed that both the Rio and the IMU are mounted such that +Z is UP
    public static final Rotation2d kRioOrientation =
        switch (getRobot()) {
          case LEONARDO -> Rotation2d.fromDegrees(90.);
          case GEORGE -> Rotation2d.fromDegrees(0.);
          default -> Rotation2d.fromDegrees(0.);
        };
    // IMU can be one of Pigeon2 or NavX
    public static final Rotation2d kIMUOrientation =
        switch (getRobot()) {
          case LEONARDO -> Rotation2d.fromDegrees(0.);
          case GEORGE -> Rotation2d.fromDegrees(0.);
          default -> Rotation2d.fromDegrees(0.);
        };
  }

  /** Operator Constants *************************************************** */
  public static class OperatorConstants {

    // Joystick Functions
    // Set to TRUE for Drive = Left Stick, Turn = Right Stick; else FALSE
    public static final boolean kDriveLeftTurnRight =
        switch (getRobot()) {
          case GEORGE -> true; // Testing
          //  case LEONARDO -> true;
          case LEONARDO -> false; // Kate's preference
          case SIMBOT -> true; // Default
        };

    // Joystick Deadbands
    public static final double kDeadband = 0.1;
    public static final double kTurnConstant = 6;

    // Joystick slew rate limiters to smooth erratic joystick motions, measured in units per second
    public static final double kJoystickSlewLimit = 0.5;

    // Override and Console Toggle Switches
    // Assumes this controller: https://www.amazon.com/gp/product/B00UUROWWK
    // Example from:
    // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736/72
    public static final int ELEVATOR_OVERRIDE = 9;
    public static final int INTAKE_OVERRIDE = 10;
    public static final int ALGAE_OVERRIDE = 11;
    public static final int VISION_OVERRIDE = 12;
  }

  /** Autonomous Action Constants ****************************************** */
  public static final class AutoConstants {

    // Drive and Turn PID constants used for PathPlanner
    public static final PIDConstants kPPdrivePID = new PIDConstants(10, 0.0, 0.0);
    // new PIDConstants(DrivebaseConstants.kDriveP, 0.0, DrivebaseConstants.kDriveD);
    public static final PIDConstants kPPsteerPID = new PIDConstants(4, 0.0, 0.0);
    // new PIDConstants(DrivebaseConstants.kSteerP, 0.0, DrivebaseConstants.kSteerD);
    // 1 Cordinate = 1 meter
    // *** 1 meter = 39.3701 inches
    // Pathplanner Maxiums with MK4i L3 with an amp limit of 40 Amps and other robot specifics
    // Max Linear Speed = 5 m/s
    // Max Angular Accel = 1354 m/s^2
    // Max Linear Accel = 9 m/s^2

    // Theroedical Maximums
    // Max Linear Speed = 5.14 m/s
    // Max Angular Accel = 1824 m/s^2
    // Max Linear Accel = 11.8 m/s^2

    // PathPlanner Config constants
    public static final RobotConfig kPathPlannerConfig =
        new RobotConfig(
            Kilograms.of(PhysicalConstants.kRobotMassKg),
            KilogramSquareMeters.of(PhysicalConstants.kRobotMOI),
            new ModuleConfig(
                Meters.of(SwerveConstants.kWheelRadiusMeters),
                MetersPerSecond.of(DrivebaseConstants.kMaxLinearSpeed),
                PhysicalConstants.kWheelCOF,
                DCMotor.getKrakenX60Foc(1).withReduction(SwerveConstants.kDriveGearRatio),
                Amps.of(SwerveConstants.kDriveSlipCurrent),
                1),
            Drive.getModuleTranslations());

    // Alternatively, we can build this from the PathPlanner GUI:
    // public static final RobotConfig kPathPlannerConfig = RobotConfig.fromGUISettings();

    // Drive and Turn PID constants used for Chorep
    public static final PIDConstants kChoreoDrivePID = new PIDConstants(10.0, 0.0, 0.0);
    public static final PIDConstants kChoreoSteerPID = new PIDConstants(7.5, 0.0, 0.0);
  }

  /** LED Constants ******************************************************** */
  public static class LEDConstants {

    // Number of LEDS
    public static final int nLED = 128;
  }

  /** Vision Constants (Assuming PhotonVision) ***************************** */
  public static class VisionConstants {

    // AprilTag Identification Constants
    public static final double kAmbiguityThreshold = 0.4;
    public static final double kTargetLogTimeSecs = 0.1;
    public static final double kFieldBorderMargin = 0.5;
    public static final double kZMargin = 0.75;
    public static final double kXYZStdDevCoefficient = 0.005;
    public static final double kThetaStdDevCoefficient = 0.01;

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static final double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  /** Vision Camera Posses ************************************************* */
  public static class Cameras {
    // Camera names, must match names configured on coprocessor
    // public static String cameraElevatorL = "Photon_BW6"; // On left side of elevator
    // public static String cameraElevatorR = "Photon_BW3"; // On right side of elevator
    // public static String cameraElevatorC = "Photon_BW3"; // On center of elevator
    // public static String cameraIntakeDown = "Photon_BW5"; // On intake churro, looking down

    public static String cameraCL = "Photon_BW1"; //  Camera in the double mount in center *Left
    public static String cameraCR = "Photon_BW2"; //  Camera in the double mount in center *Right
    public static String cameraIntake = "Photon_BW4"; // Camera facing up from the ground intake

    // Incorrect distance measurement factor
    public static double BW1Stretch = 1.00; // 1.02;
    public static double BW2Stretch = 1.00; // 1.01;
    public static double BW4Stretch = 1.00; // 1.06;

    // ... And more, if needed

    // Robot to camera transforms
    // public static Transform3d robotToCameraEL =
    //     new Transform3d(
    //         Units.inchesToMeters(-12.3),
    //         Units.inchesToMeters(-(9.23 - 0.375)), // Waiting for actual mounts
    //         Units.inchesToMeters(10.1 - 1.0), // before adding these...
    //         new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0 + 20.0))
    //             .rotateBy(new Rotation3d(0.0, Units.degreesToRadians(25.0), 0.0)));

    // public static Transform3d robotToCameraER =
    //     new Transform3d(
    //         Units.inchesToMeters(-12.3),
    //         Units.inchesToMeters(9.23 - 0.375),
    //         Units.inchesToMeters(10.1 - 1.0),
    //         new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0 - 20.0))
    //             .rotateBy(new Rotation3d(0.0, Units.degreesToRadians(25.0), 0.0)));

    // For the double camera mount in the center * Right side
    public static Transform3d robotToCameraECL =
        new Transform3d(
            Units.inchesToMeters(-13.818 + .8125),
            Units.inchesToMeters(0.0 + .875),
            Units.inchesToMeters(6.122),
            new Rotation3d(0.0, Units.degreesToRadians(10.0), Units.degreesToRadians(180.0)));
    // For the double camera mount in the center * Left side
    public static Transform3d robotToCameraECR =
        new Transform3d(
            Units.inchesToMeters(-13.818 + .8125),
            Units.inchesToMeters(0.0 - .875),
            Units.inchesToMeters(6.122),
            new Rotation3d(0.0, Units.degreesToRadians(10.0), Units.degreesToRadians(180.0)));

    // public static Transform3d robotToCameraEC =
    //     new Transform3d(
    //         Units.inchesToMeters(-13.818),
    //         Units.inchesToMeters(0.0),
    //         Units.inchesToMeters(6.122),
    //         new Rotation3d(0.0, Units.degreesToRadians(25.0), Units.degreesToRadians(180.0)));
    public static Transform3d robotToCameraIntake =
        new Transform3d(
            Units.inchesToMeters(13.613),
            Units.inchesToMeters(3.5),
            Units.inchesToMeters(6.122),
            new Rotation3d(0.0, Units.degreesToRadians(25.0), 0.0));

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera EL
          1.0, // Camera ER
          2.0 // Camera ID
        };
  }

  /** List of Device CAN and Power Distribution Circuit IDs **************** */
  public static class CANandPowerPorts {

    /* DRIVETRAIN CAN DEVICE IDS */
    // Input the correct Power Distribution Module port for each motor!!!!

    // Front Left
    public static final RobotDeviceId FL_DRIVE =
        new RobotDeviceId(SwerveConstants.kFLDriveMotorId, SwerveConstants.kFLDriveCanbus, 7);
    public static final RobotDeviceId FL_ROTATION =
        new RobotDeviceId(SwerveConstants.kFLSteerMotorId, SwerveConstants.kFLSteerCanbus, 6);
    public static final RobotDeviceId FL_CANCODER =
        new RobotDeviceId(SwerveConstants.kFLEncoderId, SwerveConstants.kFLEncoderCanbus, 5);
    // Front Right
    public static final RobotDeviceId FR_DRIVE =
        new RobotDeviceId(SwerveConstants.kFRDriveMotorId, SwerveConstants.kFRDriveCanbus, 2);
    public static final RobotDeviceId FR_ROTATION =
        new RobotDeviceId(SwerveConstants.kFRSteerMotorId, SwerveConstants.kFRSteerCanbus, 3);
    public static final RobotDeviceId FR_CANCODER =
        new RobotDeviceId(SwerveConstants.kFREncoderId, SwerveConstants.kFREncoderCanbus, 4);
    // Back Left
    public static final RobotDeviceId BL_DRIVE =
        new RobotDeviceId(SwerveConstants.kBLDriveMotorId, SwerveConstants.kBLDriveCanbus, 12);
    public static final RobotDeviceId BL_ROTATION =
        new RobotDeviceId(SwerveConstants.kBLSteerMotorId, SwerveConstants.kBLSteerCanbus, 13);
    public static final RobotDeviceId BL_CANCODER =
        new RobotDeviceId(SwerveConstants.kBLEncoderId, SwerveConstants.kBLEncoderCanbus, 14);
    // Back Right
    public static final RobotDeviceId BR_DRIVE =
        new RobotDeviceId(SwerveConstants.kBRDriveMotorId, SwerveConstants.kBRSteerCanbus, 17);
    public static final RobotDeviceId BR_ROTATION =
        new RobotDeviceId(SwerveConstants.kBRSteerMotorId, SwerveConstants.kBRSteerCanbus, 16);
    public static final RobotDeviceId BR_CANCODER =
        new RobotDeviceId(SwerveConstants.kBREncoderId, SwerveConstants.kBREncoderCanbus, 15);
    // Pigeon
    public static final RobotDeviceId PIGEON =
        new RobotDeviceId(SwerveConstants.kPigeonId, SwerveConstants.kCANbusName, null);

    /* SUBSYSTEM CAN DEVICE IDS */
    public static final RobotDeviceId ELEVATOR = new RobotDeviceId(11, "", 18);
    public static final RobotDeviceId CORAL_MECH = new RobotDeviceId(16, "", 19);
    public static final RobotDeviceId INTAKE_PIVOT = new RobotDeviceId(22, "", 0);
    public static final RobotDeviceId INTAKE_ROLLER = new RobotDeviceId(21, "", 1);
    public static final RobotDeviceId INTAKE_ENCODER = new RobotDeviceId(23, "", null);
    public static final RobotDeviceId ALGAE_PIVOT = new RobotDeviceId(26, "", 10);
    public static final RobotDeviceId ALGAE_ROLLER = new RobotDeviceId(27, "", 11);
    public static final RobotDeviceId CLIMB = new RobotDeviceId(31, "", 8);
    public static final RobotDeviceId LED = new RobotDeviceId(36, "", null);

    // public static final RobotDeviceId Coral_Scorer = new RobotDeviceId(16, "", 9);

    /* BEAM BREAK and/or LIMIT SWITCH DIO CHANNELS */
    // This is where digital I/O feedback devices are defined
    // Example:
    // public static final int ELEVATOR_BOTTOM_LIMIT = 3;
    public static final int ELEVATOR_BOTTOM_LIMIT = 0;
    public static final int ALGAE_PIVOT_ENCODER = 1;
    public static final int CORAL_LIGHT_STOP = 2;
    public static final int CLIMB_PIVOT_ENCODER = 3;

    /* LINEAR SERVO PWM CHANNELS */
    // This is where PWM-controlled devices (actuators, servos, pneumatics, etc.)
    // are defined
    // Example:
    public static final int CLIMB_SERVO = 0;
  }

  public static class DriveToPositionConstatnts {

    // The robot is facing AWAY from the tag, so its pose angle matches that of the tag.
    // Scoring position has the bumpers 3" from the tag.  Bumper-to-center distance is 18", ergo the
    // robot pose is 21" from the tag.
    public static Translation2d kLeftReefPost =
        new Translation2d(Units.inchesToMeters(17.5), Units.inchesToMeters(-6.75));
    // public static Translation2d kLeftReefPostClose =
    //     new Translation2d(Units.inchesToMeters(16.75), Units.inchesToMeters(-6.75));
    public static Translation2d kRightReefPost =
        new Translation2d(Units.inchesToMeters(17.5), Units.inchesToMeters(+6.75));
    public static Translation2d kRightReefPostClose =
        new Translation2d(Units.inchesToMeters(16.75), Units.inchesToMeters(+6.75));
    public static Translation2d kAlgaeGrab =
        new Translation2d(Units.inchesToMeters(26.0), Units.inchesToMeters(0.0));

    public static Translation2d kProcessor =
        new Translation2d(Units.inchesToMeters(26.0), Units.inchesToMeters(0.0));

    public static Translation2d kStation =
        new Translation2d(Units.inchesToMeters(18.0), Units.inchesToMeters(0.0));
  }

  /** AprilTag Field Layout ************************************************ */
  /* SEASON SPECIFIC! -- This section is for 2025 (Reefscape) */
  public static class AprilTagConstants {

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final String aprilTagFamily = "36h11";
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    @Getter
    public enum AprilTagLayoutType {
      OFFICIAL("2025-official");

      private AprilTagLayoutType(String name) {
        if (Constants.disableHAL) {
          layout = null;
        } else {
          try {
            layout =
                new AprilTagFieldLayout(
                    Path.of(
                        Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
          } catch (IOException e) {
            throw new RuntimeException(e);
          }
        }
        if (layout == null) {
          layoutString = "";
        } else {
          try {
            layoutString = new ObjectMapper().writeValueAsString(layout);
          } catch (JsonProcessingException e) {
            throw new RuntimeException(
                "Failed to serialize AprilTag layout JSON " + toString() + "for PhotonVision");
          }
        }
      }

      private final AprilTagFieldLayout layout;
      private final String layoutString;
    }
  }

  /** Deploy Directoy Location Constants *********************************** */
  public static final class DeployConstants {
    public static final String apriltagDir = "apriltags";
    public static final String choreoDir = "choreo";
    public static final String pathplannerDir = "pathplanner";
    public static final String yagslDir = "swerve";
  }

  /***************************************************************************/
  /** Getter functions -- do not modify ************************************ */
  /** Get the current robot */
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
          .set(true);
      robotType = RobotType.LEONARDO;
    }
    return robotType;
  }

  /** Get the current robot mode */
  public static Mode getMode() {
    return switch (robotType) {
      case GEORGE, LEONARDO -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  /** Get the current swerve drive type */
  public static SwerveType getSwerveType() {
    return swerveType;
  }

  /** Get the current autonomous path planning type */
  public static AutoType getAutoType() {
    return autoType;
  }

  /** Get the current autonomous path planning type */
  public static VisionType getVisionType() {
    return visionType;
  }

  /** Get the current CTRE/Phoenix Pro License state */
  public static CTREPro getPhoenixPro() {
    return phoenixPro;
  }

  /** Get the current AprilTag layout type. */
  public static AprilTagLayoutType getAprilTagLayoutType() {
    return AprilTagConstants.defaultAprilTagType;
  }
}
