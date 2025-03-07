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

package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.LEDConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * LED control class
 *
 * <p>This LED subsystem is based on the VirtualSubsystem, which runs all the time regardless of
 * enabled state of the robot.
 */
public class LED extends VirtualSubsystem {
  private CANdle candle = new CANdle(CANandPowerPorts.LED.getDeviceNumber());
  private CANdleConfiguration config = new CANdleConfiguration();
  private boolean red;
  private double i = 0;
  private static double lightReset;

  TwinkleOffAnimation twinkleR =
      new TwinkleOffAnimation(255, 0, 0, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
  TwinkleOffAnimation twinkleG =
      new TwinkleOffAnimation(0, 255, 0, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
  TwinkleOffAnimation twinkleB =
      new TwinkleOffAnimation(0, 0, 255, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
  TwinkleOffAnimation twinkleY =
      new TwinkleOffAnimation(255, 165, 0, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
  TwinkleOffAnimation twinkleO =
      new TwinkleOffAnimation(255, 80, 0, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
  TwinkleOffAnimation twinkleP =
      new TwinkleOffAnimation(48, 25, 52, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
  TwinkleOffAnimation twinkleFG =
      new TwinkleOffAnimation(
          ((int) Color.kForestGreen.red * 255),
          ((int) Color.kForestGreen.green * 255),
          ((int) Color.kForestGreen.blue * 255),
          0,
          .01,
          LEDConstants.nLED,
          TwinkleOffPercent.Percent100);
  TwinkleOffAnimation twinkleCoral =
      new TwinkleOffAnimation(
          ((int) Color.kCoral.red * 255),
          ((int) Color.kCoral.green * 255),
          ((int) Color.kCoral.blue * 255),
          0,
          .01,
          LEDConstants.nLED,
          TwinkleOffPercent.Percent100);
  private TwinkleOffAnimation col1 = twinkleO;
  private TwinkleOffAnimation col2 = twinkleP;

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kGold;
  private Color secondaryDisabledColor = Color.kDarkBlue;

  private boolean estopped = false;
  private static boolean elevatorEstopped = false;
  private static boolean intakeEstopped = false;
  private static boolean algaemechEstopped = false;
  private static boolean coralReady = true;

  private static LED instance;

  /** Return an instance of this class */
  public static LED getInstance() {
    if (instance == null) {
      instance = new LED();
    }
    return instance;
  }

  /** Constructor */
  private LED() {
    coralReady = true;
    red = false;
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    candle.configAllSettings(config);
  }

  /** Periodic function called every robot cycle */
  public synchronized void periodic() {

    // Log the execution time
    long start = System.nanoTime();

    // Update alliance color
    // if (DriverStation.isFMSAttached()) {
    alliance = DriverStation.getAlliance();
    allianceColor =
        alliance
            .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
            .orElse(Color.kGold);
    secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : Color.kDarkBlue;
    //  }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Select LED mode =====================================================
    // off(); // Default to off

    if (estopped) {
      // E-STOP
      solid(Color.kRed);
    } else if (DriverStation.isDisabled()) {
      // When disabled
      pickTwoTwinkle();
    } else if (DriverStation.isAutonomous()) {
      if (coralReady) {

        candle.animate(new RainbowAnimation(.5, 1, LEDConstants.nLED));
      } else {
        solid(allianceColor);
      }
    } else {
      // Teleop Enabled
      if (coralReady) {
        solid(Color.kWhite);
      } else {
        solid(allianceColor);
        // setRGB(255, 255, 255);
      }

      // solid(Color.kForestGreen);
    }

    // VARIOUS ALERT OR SCORING MODES
    // Elevator estop alert
    // if (elevatorEstopped) {
    //   solid(Color.kRed);
    // }
    // // Intake estop alert
    // if (intakeEstopped) {
    //   solid(Color.kRed);
    // }
    // // AlgaeMech estop alert
    // if (algaemechEstopped) {
    //   solid(Color.kRed);
    // }

    // Quick logging to see how long this periodic takes
    long finish = System.nanoTime();
    long timeElapsed = finish - start;
    Logger.recordOutput("LoggedRobot/LEDCodeMS", (double) timeElapsed / 1.e6);
  }

  /** Return something about the reset of the lights */
  private boolean getLightReset() {
    boolean reset;
    if (candle.getCurrent() < .2 && lightReset > .7) {
      reset = true;
    } else {
      reset = false;
    }
    lightReset = candle.getCurrent();
    return reset;
  }

  /** Run the lights in a raimbow twinkle pattern */
  public void rainbowTwinkle() {
    double rand = Math.random();

    // FireAnimation fire = new FireAnimation(1, .5, LEDConstants.nLED, .9, .5);
    // candle.setLEDs(0, 255, 0, 0, 0, LEDConstants.nLED);
    // candle.setLEDs(255, 255, 255, 0, 8, 44);
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, .5, LEDConstants.nLED);
    if (rand < .166) {
      candle.animate(twinkleR);
    } else if (rand < .332) {
      candle.animate(twinkleG);
    } else if (rand < .498) {
      candle.animate(twinkleB);
    } else if (rand < .664) {
      candle.animate(twinkleY);
    } else if (rand < .830) {
      candle.animate(twinkleO);
    } else {
      candle.animate(twinkleP);
    }
    // candle.animate(rainbowAnim);
  }

  /** Run the lights in a random 2-color twinkle */
  public void pickTwoTwinkle() {

    double randLightSet = Math.random();

    if (randLightSet < .5) {
      candle.animate(col1);
    } else {
      candle.animate(col2);
    }
  }

  /** Turn the lights off */
  private void off() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 0, 0, 0, LEDConstants.nLED);
  }

  /** Set the lights to a specific RGB color */
  private void setRGB(int r, int g, int b) {
    candle.clearAnimation(0);
    candle.setLEDs(r, g, b, 0, 0, LEDConstants.nLED);
  }

  /** Get the current consumed by the CANdle */
  private double getCurrent() {
    return candle.getCurrent();
  }

  /** Set the lights to a solid color */
  private void solid(Color color) {
    candle.clearAnimation(0);
    if (color != null) {
      candle.setLEDs(
          (int) color.red * 255,
          (int) color.green * 255,
          (int) color.blue * 255,
          0,
          0,
          LEDConstants.nLED);
    }
  }

  /** Set the lights to the color of a CORAL */
  private void coralLarson(Color allianceColor) {
    candle.clearAnimation(0);

    candle.setLEDs(255, 255, 255, 0, 0, LEDConstants.nLED);
  }

  /** Indicate that the robot is ready to score */
  private void scoreReady() {
    candle.clearAnimation(0);
    candle.setLEDs(255, 165, 0, 0, 0, LEDConstants.nLED);
  }

  /** Indicate that the robot is not ready to score */
  private void scoreNotReady() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 255, 0, 0, LEDConstants.nLED);
  }

  /** Get the ELEVATOR E-STOP status */
  public static boolean getElevatorEStop() {
    return elevatorEstopped;
  }

  /** Get the INTAKE E-STOP status */
  public static boolean getIntakeEStop() {
    return intakeEstopped;
  }

  /* Get the ALGAE E-STOP status */
  public static boolean getAgaeMechEStop() {
    return algaemechEstopped;
  }

  /**
   * Set the ELEVATOR E-STOP status
   *
   * @param eStopSwitch The boolean value of the e-stop switch
   */
  public static void setElevatorEStop(boolean eStopSwitch) {
    elevatorEstopped = eStopSwitch;
  }

  /**
   * Set the INTAKE E-STOP status
   *
   * @param eStopSwitch The boolean value of the e-stop switch
   */
  public static void setIntakeEStop(boolean eStopSwitch) {
    intakeEstopped = eStopSwitch;
  }

  /**
   * Set the ALGAE E-STOP status
   *
   * @param eStopSwitch The boolean value of the e-stop switch
   */
  public static void setAgaeMechEStop(boolean eStopSwitch) {
    algaemechEstopped = eStopSwitch;
  }

  /**
   * Set the CORAL score ready status
   *
   * @param coralScoreReady The boolean value of the CORAL score-ready indicator
   */
  public static void setCoralReady(boolean coralScoreReady) {
    coralReady = coralScoreReady;
  }
}
