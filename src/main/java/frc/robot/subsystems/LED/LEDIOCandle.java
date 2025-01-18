package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.Timer;

public class LEDIOCandle implements LEDIO {
  CANdle candle = new CANdle(1);
  CANdleConfiguration config = new CANdleConfiguration();
  Orchestra orc = new Orchestra();
  private boolean red;
  private double i = 0;

  public LEDIOCandle() {
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
  public void larson() {

    LarsonAnimation gary = new LarsonAnimation(255, 0, 0, 0, .5, 67, BounceMode.Back, 67, 8);
    while (true) {
      candle.animate(gary);
    }
  }

  @Override
  public void police() {
    candle.clearAnimation(0);
    if (red) {
      candle.setLEDs(255, 0, 0, 0, 0, 35);
      candle.setLEDs(0, 0, 0, 0, 35, 67);
      if (Timer.getFPGATimestamp() - i > .5) {
        red = false;
        i = Timer.getFPGATimestamp();
      }
    }

    if (red == false) {
      candle.setLEDs(0, 0, 255, 0, 35, 67);
      candle.setLEDs(0, 0, 0, 0, 0, 35);
      if (Timer.getFPGATimestamp() - i > .5) {
        red = true;
        i = Timer.getFPGATimestamp();
      }
    }
  }
  // set the CANdle LEDs to white

  // create a rainbow animation:
  // - max brightness
  // - half speed
  // - 64 LEDs
  //  RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
  //  candle.animate(rainbowAnim);

}
