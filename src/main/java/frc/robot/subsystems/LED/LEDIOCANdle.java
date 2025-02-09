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

public class LEDIOCANdle implements LEDIO {
  CANdle candle = new CANdle(1);
  CANdleConfiguration config = new CANdleConfiguration();
  private boolean red;
  private double i = 0;

  public LEDIOCANdle() {
    red = false;
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    candle.configAllSettings(config);
  }

  @Override
  public void rainbowTwinkle() {
    double rand = Math.random();

    TwinkleOffAnimation twinkleR =
        new TwinkleOffAnimation(255, 0, 0, 0, .01, 67, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleG =
        new TwinkleOffAnimation(0, 255, 0, 0, .01, 67, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleB =
        new TwinkleOffAnimation(0, 0, 255, 0, .01, 67, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleY =
        new TwinkleOffAnimation(255, 255, 0, 0, .01, 67, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleO =
        new TwinkleOffAnimation(255, 165, 0, 0, .01, 67, TwinkleOffPercent.Percent100);
    TwinkleOffAnimation twinkleP =
        new TwinkleOffAnimation(48, 25, 52, 0, .01, 67, TwinkleOffPercent.Percent100);
    // FireAnimation fire = new FireAnimation(1, .5, 67, .9, .5);
    // candle.setLEDs(0, 255, 0, 0, 0, 67);
    // candle.setLEDs(255, 255, 255, 0, 8, 44);
    // RainbowAnimation rainbowAnim = new RainbowAnimation(1, .5, 67);
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

  @Override
  public void off() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 0, 0, 0, 67);
  }

  @Override
  public void scoreReady() {
    candle.clearAnimation(0);
    candle.setLEDs(255, 165, 0, 0, 0, 67);
  }

  @Override
  public void scoreNotReady() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 255, 0, 0, 67);
  }
}
