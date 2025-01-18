package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

public class LEDIOCandle implements LEDIO {
  CANdle candle = new CANdle(1);
  CANdleConfiguration config = new CANdleConfiguration();

  public LEDIOCandle() {
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    candle.configAllSettings(config);
  }

  @Override
  public void m_animate() {

    // candle.setLEDs(255, 255, 255);
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, .5, 67);
    candle.animate(rainbowAnim);
  }
  // set the CANdle LEDs to white

  // create a rainbow animation:
  // - max brightness
  // - half speed
  // - 64 LEDs
  //  RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
  //  candle.animate(rainbowAnim);

}
