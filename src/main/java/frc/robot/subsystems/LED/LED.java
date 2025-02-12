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
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.LEDConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.Optional;

public class LED extends VirtualSubsystem {
  private CANdle candle = new CANdle(CANandPowerPorts.LED.getDeviceNumber());
  private CANdleConfiguration config = new CANdleConfiguration();
  private boolean red;
  private double i = 0;

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kGold;
  private Color secondaryDisabledColor = Color.kDarkBlue;

  private boolean estopped = false;
  private boolean elevatorEstopped = false;
  private boolean intakeEstopped = false;
  private boolean algaemechEstopped = false;
  private boolean coralReady = false;

  private static LED instance;

  public static LED getInstance() {
    if (instance == null) {
      instance = new LED();
    }
    return instance;
  }

  private LED() {
    red = false;
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    candle.configAllSettings(config);
  }

  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(Color.kGold);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : Color.kDarkBlue;
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Select LED mode =====================================================
    off(); // Default to off

    if (estopped) {
      // E-STOP
      solid(Color.kRed);
    } else if (DriverStation.isDisabled()) {
      // When disabled
      rainbowTwinkle();
    } else if (DriverStation.isAutonomous()) {
      // Autonomous Mode
      solid(Color.kBrown);
    } else {
      // Teleop Enabled
      solid(Color.kForestGreen);
    }

    // VARIOUS ALERT OR SCORING MODES
    // Elevator estop alert
    if (elevatorEstopped) {
      solid(Color.kRed);
    }
    // Intake estop alert
    if (intakeEstopped) {
      solid(Color.kRed);
    }
    // AlgaeMech estop alert
    if (algaemechEstopped) {
      solid(Color.kRed);
    }
  }

  public void rainbowTwinkle() {
    double rand = Math.random();

    TwinkleOffAnimation twinkleR =
        new TwinkleOffAnimation(255, 0, 0, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleG =
        new TwinkleOffAnimation(0, 255, 0, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleB =
        new TwinkleOffAnimation(0, 0, 255, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleY =
        new TwinkleOffAnimation(
            255, 255, 0, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleO =
        new TwinkleOffAnimation(
            255, 165, 0, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleP =
        new TwinkleOffAnimation(
            48, 25, 52, 0, .01, LEDConstants.nLED, TwinkleOffPercent.Percent100);
    // FireAnimation fire = new FireAnimation(1, .5, LEDConstants.nLED, .9, .5);
    // candle.setLEDs(0, 255, 0, 0, 0, LEDConstants.nLED);
    // candle.setLEDs(255, 255, 255, 0, 8, 44);
    // RainbowAnimation rainbowAnim = new RainbowAnimation(1, .5, LEDConstants.nLED);
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
  }

  private void off() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 0, 0, 0, LEDConstants.nLED);
  }

  private void solid(Color color) {
    if (color != null) {
      candle.clearAnimation(0);
      candle.setLEDs(
          (int) color.red * 255,
          (int) color.green * 255,
          (int) color.blue * 255,
          0,
          0,
          LEDConstants.nLED);
    }
  }

  private void scoreReady() {
    candle.clearAnimation(0);
    candle.setLEDs(255, 165, 0, 0, 0, LEDConstants.nLED);
  }

  private void scoreNotReady() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 255, 0, 0, LEDConstants.nLED);
  }

  public boolean getElevatorEStop() {
    return elevatorEstopped;
  }

  public boolean getIntakeEStop() {
    return intakeEstopped;
  }

  public boolean getAgaeMechEStop() {
    return algaemechEstopped;
  }

  public void setElevatorEStop(boolean eStopSwitch) {
    elevatorEstopped = eStopSwitch;
  }

  public void setIntakeEStop(boolean eStopSwitch) {
    intakeEstopped = eStopSwitch;
  }

  public void setAgaeMechEStop(boolean eStopSwitch) {
    algaemechEstopped = eStopSwitch;
  }

  public void setCoralReady(boolean coralReady) {
    this.coralReady = coralReady;
  }
}
