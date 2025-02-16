// Copyright (c) 2025 FRC 2486
// http://github.com/Coconuts2486-FRC
// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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

import static frc.robot.Constants.Cameras.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AprilTagConstants.AprilTagLayoutType;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.accelerometer.Accelerometer;
import frc.robot.subsystems.algae_mech.AlgaeMech;
import frc.robot.subsystems.algae_mech.AlgaeMechIO;
import frc.robot.subsystems.algae_mech.AlgaeMechIOTalonFX;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.climber.ClimbIO;
import frc.robot.subsystems.climber.ClimbIOTalonFX;
import frc.robot.subsystems.coral_mech.CoralScorer;
import frc.robot.subsystems.coral_mech.CoralScorerIO;
import frc.robot.subsystems.coral_mech.CoralScorerIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.state_keeper.ReefTarget;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.GetJoystickValue;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OverrideSwitches;
import frc.robot.util.PowerMonitoring;
import frc.robot.util.RBSIEnum;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** This is the location for defining robot hardware, commands, and controller button bindings. */
public class RobotContainer {

  // **** This is a Pathplanner On-the-Fly Command ****/
  // Create a list of waypoints from poses. Each pose represents one waypoint.
  // The rotation component of the pose should be the direction of travel. Do not use
  // holonomic rotation.
  List<Waypoint> woahpoints =
      PathPlannerPath.waypointsFromPoses(
          new Pose2d(8.180, 6.184, Rotation2d.fromDegrees(0)),
          new Pose2d(9.4, 6.184, Rotation2d.fromDegrees(0)));

  PathConstraints constraints =
      new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
  // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can
  // also use unlimited constraints, only limited by motor torque and nominal battery
  // voltage

  // Create the path using the waypoints created above
  PathPlannerPath woah =
      new PathPlannerPath(
          woahpoints,
          constraints,
          null, // The ideal starting state, this is only relevant for pre-planned paths,
          // so
          // can be null for on-the-fly paths.
          new GoalEndState(
              0.0,
              Rotation2d.fromDegrees(
                  180)) // Goal end state. You can set a holonomic rotation here. If
          // using a
          // differential drivetrain, the rotation will have no effect.
          );

  // Prevent the path from being flipped if the coordinates are already correct

  /** Define the Driver and, optionally, the Operator/Co-Driver Controllers */
  // Replace with ``CommandPS4Controller`` or ``CommandJoystick`` if needed
  public final CommandXboxController driverController = new CommandXboxController(0); // Main Driver

  public final CommandXboxController operatorController =
      new CommandXboxController(1); // Second Operator
  public final OverrideSwitches overrides = new OverrideSwitches(2); // Console toggle switches

  // Define Triggers
  private Trigger leftBumper = driverController.leftBumper();
  private Trigger rightBumper = driverController.rightBumper();
  private final Trigger elevatorDisable = overrides.Switch(OperatorConstants.ELEVATOR_OVERRIDE);
  private final Trigger intakePivotDisable = overrides.Switch(OperatorConstants.INTAKE_OVERRIDE);
  private final Trigger algaePivotDisable = overrides.Switch(OperatorConstants.ALGAE_OVERRIDE);
  private final Trigger visionOdometryDisable = overrides.Switch(OperatorConstants.VISION_OVERRIDE);

  //   private final Alert driverDisconnected =
  //       new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  //   private final Alert operatorDisconnected =
  //       new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  //   private final Alert overrideDisconnected =
  //       new Alert("Override controller disconnected (port 5).", AlertType.kInfo);

  /** Declare the robot subsystems here ************************************ */
  // These are the "Active Subsystems" that the robot controlls
  private final Drive m_drivebase;

  private final Elevator m_elevator;
  private final CoralScorer m_coralScorer;
  private final Intake m_intake;
  private final AlgaeMech m_algaeMech;
  private final Climb m_climber;

  // These are "Virtual Subsystems" that report information but have no motors
  private final Accelerometer m_accel;
  private final ReefTarget m_reefTarget = ReefTarget.getInstance();
  private final Vision m_vision;
  private final PowerMonitoring m_power;
  private final LED m_led = LED.getInstance();

  /** Dashboard inputs ***************************************************** */
  // AutoChoosers for both supported path planning types
  private final LoggedDashboardChooser<Command> autoChooserPathPlanner;

  private final AutoChooser autoChooserChoreo;
  private final AutoFactory autoFactoryChoreo;
  // Input estimated battery capacity (if full, use printed value)
  private final LoggedTunableNumber batteryCapacity =
      new LoggedTunableNumber("Battery Amp-Hours", 18.0);

  // Alerts
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);

  /**
   * Constructor for the Robot Container. This container holds subsystems, opertator interface
   * devices, and commands.
   */
  public RobotContainer() {
    woah.preventFlipping = true;
    // Instantiate Robot Subsystems based on RobotType
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_drivebase = new Drive();
        m_elevator = new Elevator(new ElevatorIOTalonFX());
        m_coralScorer = new CoralScorer(new CoralScorerIOTalonFX());
        m_intake = new Intake(new IntakeIOTalonFX());
        m_algaeMech = new AlgaeMech(new AlgaeMechIOTalonFX());
        m_climber = new Climb(new ClimbIOTalonFX());

        // Virtual Subsystems
        m_vision =
            switch (Constants.getVisionType()) {
              case PHOTON ->
                  new Vision(
                      m_drivebase::addVisionMeasurement,
                      new VisionIOPhotonVision(camera0Name, robotToCamera0),
                      new VisionIOPhotonVision(camera1Name, robotToCamera1));
              case LIMELIGHT ->
                  new Vision(
                      m_drivebase::addVisionMeasurement,
                      new VisionIOLimelight(camera0Name, m_drivebase::getRotation),
                      new VisionIOLimelight(camera1Name, m_drivebase::getRotation));
              case NONE ->
                  new Vision(
                      m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
              default -> null;
            };
        m_accel = new Accelerometer(m_drivebase.getGyro());
        // m_coralState = new CoralState();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drivebase = new Drive();
        m_elevator = new Elevator(new ElevatorIO() {}); // make elevator Io sim
        m_coralScorer = new CoralScorer(new CoralScorerIO() {});
        m_intake = new Intake(new IntakeIO() {});
        m_algaeMech = new AlgaeMech(new AlgaeMechIO() {});
        m_climber = new Climb(new ClimbIO() {});

        // Virtual Subsystems
        m_vision =
            new Vision(
                m_drivebase::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_drivebase::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_drivebase::getPose));
        m_accel = new Accelerometer(m_drivebase.getGyro());
        // m_coralState = new CoralState();
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drivebase = new Drive();
        m_elevator = new Elevator(new ElevatorIO() {});
        m_coralScorer = new CoralScorer(new CoralScorerIO() {});
        m_intake = new Intake(new IntakeIO() {});
        m_algaeMech = new AlgaeMech(new AlgaeMechIO() {});
        m_climber = new Climb(new ClimbIO() {});

        // Virtual Subsystems
        m_vision =
            new Vision(m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        m_accel = new Accelerometer(m_drivebase.getGyro());
        // m_coralState = new CoralState();
        break;
    }

    NamedCommands.registerCommand(
        "L4",
        Commands.parallel(
            new ElevatorCommand(
                ElevatorConstants.kL4,
                ElevatorConstants.kAcceleration,
                ElevatorConstants.kVelocity,
                m_elevator),
            Commands.run(() -> m_coralScorer.setCoralPercent(.0), m_coralScorer)
                .withTimeout(1)
                .andThen(Commands.run(() -> m_coralScorer.setCoralPercent(.50), m_coralScorer))));
    NamedCommands.registerCommand(
        "L3",
        // new ElevatorCommand(
        //     ElevatorConstants.kL3,
        //     ElevatorConstants.kAcceleration,
        //     ElevatorConstants.kVelocity,
        //     m_elevator));
        Commands.print("L3"));
    NamedCommands.registerCommand(
        "L2",
        // new ElevatorCommand(
        //     ElevatorConstants.kL2,
        //     ElevatorConstants.kAcceleration,
        //     ElevatorConstants.kVelocity,
        //     m_elevator));
        Commands.print("L2"));
    NamedCommands.registerCommand(
        "Bottom",
        new ElevatorCommand(
                ElevatorConstants.kElevatorZeroHeight,
                ElevatorConstants.kAcceleration.div(2.0),
                ElevatorConstants.kVelocity.div(2.0),
                m_elevator)
            .until(m_elevator::getBottomStop));
    NamedCommands.registerCommand(
        "CoralIntake", (Commands.run(() -> m_coralScorer.automaticIntake(), m_coralScorer)));

    // In addition to the initial battery capacity from the Dashbaord, ``PowerMonitoring`` takes all
    // the non-drivebase subsystems for which you wish to have power monitoring; DO NOT include
    // ``m_drivebase``, as that is automatically monitored.
    m_power =
        new PowerMonitoring(
            batteryCapacity, m_elevator, m_coralScorer, m_intake, m_algaeMech, m_climber);

    // Set up the SmartDashboard Auto Chooser based on auto type
    switch (Constants.getAutoType()) {
      case PATHPLANNER:
        autoChooserPathPlanner =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set the others to null
        autoChooserChoreo = null;
        autoFactoryChoreo = null;
        break;

      case CHOREO:
        autoFactoryChoreo =
            new AutoFactory(
                m_drivebase::getPose, // A function that returns the current robot pose
                m_drivebase::resetOdometry, // A function that resets the current robot pose to the
                // provided Pose2d
                m_drivebase::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                m_drivebase // The drive subsystem
                );
        autoChooserChoreo = new AutoChooser();
        autoChooserChoreo.addRoutine("twoPieceAuto", this::twoPieceAuto);
        // Set the others to null
        autoChooserPathPlanner = null;
        break;

      default:
        // Then, throw the error
        throw new RuntimeException(
            "Incorrect AUTO type selected in Constants: " + Constants.getAutoType());
    }

    // Set up subsystem overrides
    m_elevator.setOverrides(elevatorDisable);
    m_intake.setOverrides(intakePivotDisable);
    m_algaeMech.setOverrides(algaePivotDisable);
    m_vision.setOverrides(visionOdometryDisable);

    // Define Auto commands
    defineAutoCommands();
    // Define SysIs Routines
    definesysIdRoutines();
    // Configure the button and trigger bindings
    configureBindings();
  }

  /** Use this method to define your Autonomous commands for use with PathPlanner / Choreo */
  private void defineAutoCommands() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {

    // Send the proper joystick input based on driver preference -- Set this in `Constants.java`
    GetJoystickValue driveStickY;
    GetJoystickValue driveStickX;
    GetJoystickValue turnStickX;
    if (OperatorConstants.kDriveLeftTurnRight) {
      driveStickY = driverController::getLeftY;
      driveStickX = driverController::getLeftX;
      turnStickX = driverController::getRightX;
    } else {
      driveStickY = driverController::getRightY;
      driveStickX = driverController::getRightX;
      turnStickX = driverController::getLeftX;
    }

    // SET STANDARD DRIVING AS DEFAULT COMMAND FOR THE DRIVEBASE
    m_drivebase.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
            m_drivebase,
            () -> -driveStickY.value(),
            () -> -driveStickX.value(),
            () -> -turnStickX.value()));

    // Driver B button :>> Drive Robot-Centric
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () ->
                    DriveCommands.robotRelativeDrive(
                        m_drivebase,
                        () -> -driveStickY.value(),
                        () -> -driveStickX.value(),
                        () -> turnStickX.value()),
                m_drivebase));

    // Driver X button :>> Stop with wheels in X-Lock position
    driverController.x().onTrue(Commands.runOnce(m_drivebase::stopWithX, m_drivebase));

    // Driver RightStick :>> Coast the elevator while pressed
    driverController
        .rightStick()
        .whileTrue(
            Commands.startEnd(() -> m_elevator.setCoast(), m_elevator::setBrake, m_elevator)
                .ignoringDisable(true));

    // Driver Right Bumper :>> Intake from the floor
    driverController
        .rightBumper()
        .whileTrue(new IntakeCommand(m_intake, 0.25, -0.35))
        .whileFalse(new IntakeCommand(m_intake, 0.9, 0).until(leftBumper));

    // Driver Left Bumper :>> Other intaking stuff?
    driverController
        .leftBumper()
        .whileTrue(
            new IntakeCommand(m_intake, 0.75, 0)
                .withTimeout(0.075)
                .andThen(new IntakeCommand(m_intake, 0.75, 0.7)))
        .whileFalse(new IntakeCommand(m_intake, 0.9, 0).until(rightBumper));

    // Driver Y button :>> Manually Re-Zero the Gyro (DO NOT USE)
    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_drivebase.resetPose(
                            new Pose2d(m_drivebase.getPose().getTranslation(), new Rotation2d())),
                    m_drivebase)
                .ignoringDisable(true));

    // Operator A Button :>> Elevator to Level

    operatorController
        .b()
        .whileTrue(
            Commands.runEnd(
                () -> m_climber.twistToPosition(ClimbConstants.startClimb),
                () -> m_climber.stop(),
                m_climber));

    operatorController
        .x()
        .whileTrue(
            Commands.runEnd(
                () -> m_climber.goUntilPosition(-.5, ClimbConstants.completeClimb),
                () -> m_climber.stop(),
                m_climber));

    operatorController
        .a()
        .whileTrue(
            Commands.parallel(
                new ElevatorCommand(
                    ElevatorConstants.kL2,
                    ElevatorConstants.kAcceleration,
                    ElevatorConstants.kVelocity,
                    m_elevator),
                Commands.run(() -> m_coralScorer.setCoralPercent(.0), m_coralScorer)
                    .withTimeout(.35)
                    .andThen(
                        Commands.run(() -> m_coralScorer.setCoralPercent(.50), m_coralScorer))));

    operatorController.start().onTrue(Commands.runOnce(() -> m_climber.rachetToggle(0), m_climber));

    operatorController
        .back()
        .onTrue(Commands.runOnce(() -> m_climber.rachetToggle(.42), m_climber));

    // Operator Y Button :>> Elevator to Lower Algae
    operatorController
        .y()
        .whileTrue(
            Commands.parallel(
                new ElevatorCommand(
                    ElevatorConstants.KAlgae1,
                    ElevatorConstants.kAcceleration,
                    ElevatorConstants.kVelocity,
                    m_elevator),
                Commands.run(() -> m_algaeMech.pivotHorizontal(), m_algaeMech)
                    .withTimeout(.35)
                    .andThen(
                        Commands.run(() -> m_algaeMech.pivotOffReef(), m_algaeMech)
                            .alongWith(Commands.run(() -> m_algaeMech.setPercent(.6)))
                            .alongWith(Commands.runOnce(() -> m_algaeMech.toggleUp(false))))));

    // Release Operator X Button :>> Pivot AlgaeMech to horizontal
    operatorController
        .y()
        .onFalse(
            Commands.runOnce(() -> m_algaeMech.pivotHorizontal(), m_algaeMech)
                .alongWith(Commands.runOnce(() -> m_algaeMech.setPercent(0))));

    // Operator Right Bumper :>> Spit out algae ball
    operatorController
        .rightBumper()
        .whileTrue(
            Commands.run(() -> m_algaeMech.pivotHorizontal(), m_algaeMech)
                .alongWith(Commands.run(() -> m_algaeMech.setPercent(-1))));

    // Release Operator Right Bumper :>> Turn off algae rollers
    operatorController
        .rightBumper()
        .onFalse(
            Commands.run(() -> m_algaeMech.pivotHorizontal(), m_algaeMech)
                .alongWith(Commands.run(() -> m_algaeMech.setPercent(0))));

    // Operator Left Bumper :>> Move the AlgaeMech to stow position
    operatorController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> m_algaeMech.toggleUp(!m_algaeMech.getToggleStow()), m_algaeMech));

    // Operator POV U-D :>> Change the intended reef coral score location L-R
    operatorController.povUp().onTrue(Commands.runOnce(() -> m_reefTarget.indexUp()));
    operatorController.povDown().onTrue(Commands.runOnce(() -> m_reefTarget.indexDown()));
    // Operator POV L-R :>> Change the intended reef coral score location L-R
    operatorController.povLeft().onTrue(Commands.runOnce(() -> m_reefTarget.indexLeft()));
    operatorController.povRight().onTrue(Commands.runOnce(() -> m_reefTarget.indexRight()));

    // .alongWith(Commands.run(() -> m_coralScorer.setCoralPercent(0), m_algaeMech))
    // .withTimeout(1)
    // .andThen(
    //     new ElevatorCommand(
    //             Inches.of(38),
    //             RotationsPerSecondPerSecond.of(40),
    //             RotationsPerSecond.of(80),
    //             m_elevator)
    //         .alongWith(
    //             Commands.run(() -> m_coralScorer.setCoralPercent(.50), m_coralScorer))));
    // driverController
    //     .a()
    //     .whileFalse(new ElevatorCommand(    Inches.of(10.9),
    //     MetersPerSecondPerSecond.of(10),
    //     MetersPerSecond.of(10), m_elevator));

    // operatorController.leftBumper().whileTrue(new ElevatorCommand(Inches.of(12),
    // MetersPerSecondPerSecond.of(), MetersPerSecond.of(), m_elevator));

    // m_CoralScorer.setDefaultCommand(
    //     Commands.run(
    //         () ->
    //             m_CoralScorer.runVolts(
    //                 driverController.getRightTriggerAxis() -
    // driverController.getLeftTriggerAxis()),
    //         m_CoralScorer));

    // ** Example Commands -- Remap, remove, or change as desired **
    // Press B button while driving --> ROBOT-CENTRIC

    // driverController
    //     .rightBumper()
    //     .onTrue(
    //         new LEDCommand(m_led, m_coralScorer::getLightStop)
    //             .ignoringDisable(true)
    //             .until(driverController.leftBumper()));
    // driverController.a().onTrue(Commands.run(() -> m_coralState.indexL()));
    // driverController.y().onTrue(Commands.run(() -> m_coralState.indexR()));

    // Press A button -> BRAKE
    // driverController
    //     .a()
    //     .whileTrue(Commands.runOnce(() -> m_drivebase.setMotorBrake(true), m_drivebase));

    // driverController.a().whileTrue(new IntakeCommand(m_intake, 0));

    // m_elevator.setDefaultCommand(
    //     Commands.run(
    //         () -> m_elevator.runVolts(driverController.getRightTriggerAxis()), m_elevator));

    // the two driver controller bumpers below make it so when you let go of either button the
    // intake pivot will go to a resting posistion
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandPathPlanner() {
    // Use the ``autoChooser`` to define your auto path from the SmartDashboard
    return autoChooserPathPlanner.get();
    // return new PathPlannerAuto("Consistancy Test");
    // return AutoBuilder.followPath(woah);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void getAutonomousCommandChoreo() {
    // Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooserChoreo);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooserChoreo.selectedCommandScheduler());
  }

  /** Set the motor neutral mode to BRAKE / COAST for T/F */
  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive = Constants.getAprilTagLayoutType() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use ("
              + Constants.getAprilTagLayoutType().toString()
              + ").");
    }
  }

  /**
   * Set up the SysID routines from AdvantageKit
   *
   * <p>NOTE: These are currently only accessible with Constants.AutoType.PATHPLANNER
   */
  private void definesysIdRoutines() {
    if (Constants.getAutoType() == RBSIEnum.AutoType.PATHPLANNER) {
      // Drivebase characterization
      autoChooserPathPlanner.addOption(
          "Drive Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive Simple FF Characterization",
          DriveCommands.feedforwardCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Forward)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Reverse)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Forward)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Reverse)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      //   // Example Flywheel SysId Characterization
      //   autoChooserPathPlanner.addOption(
      //       "Flywheel SysId (Quasistatic Forward)",
      //       m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      //   autoChooserPathPlanner.addOption(
      //       "Flywheel SysId (Quasistatic Reverse)",
      //       m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      //   autoChooserPathPlanner.addOption(
      //       "Flywheel SysId (Dynamic Forward)",
      //       m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
      //   autoChooserPathPlanner.addOption(
      //       "Flywheel SysId (Dynamic Reverse)",
      //       m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }
  }

  /**
   * Example Choreo auto command
   *
   * <p>NOTE: This would normally be in a spearate file.
   */
  private AutoRoutine twoPieceAuto() {
    AutoRoutine routine = autoFactoryChoreo.newRoutine("twoPieceAuto");

    // Load the routine's trajectories
    AutoTrajectory pickupTraj = routine.trajectory("pickupGamepiece");
    AutoTrajectory scoreTraj = routine.trajectory("scoreGamepiece");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(pickupTraj.resetOdometry(), pickupTraj.cmd()));

    // Starting at the event marker named "intake", run the intake
    // pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());

    // When the trajectory is done, start the next trajectory
    pickupTraj.done().onTrue(scoreTraj.cmd());

    // While the trajectory is active, prepare the scoring subsystem
    // scoreTraj.active().whileTrue(scoringSubsystem.getReady());

    // When the trajectory is done, score
    // scoreTraj.done().onTrue(scoringSubsystem.score());

    return routine;
  }
}
