package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;
import frc.robot.util.VirtualSubsystem;

public class LEDPWM extends VirtualSubsystem {

  private static LEDPWM instance;
  public AddressableLED ledStrip; // = new AddressableLED(1);
  public AddressableLEDBuffer ledBuffer;

  /** Return an instance of this class */
  public static LEDPWM getInstance() {
    if (instance == null) {
      instance = new LEDPWM();
    }
    return instance;
  }

  public LEDPWM() {
    ledStrip = new AddressableLED(1);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.nLED);
    ledStrip.setLength(LEDConstants.nLED);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  @Override
  public void periodic() {
    solid(Color.kBlue);

    // Create an LED pattern that sets the entire strip to solid red
    // LEDPattern red = LEDPattern.solid(Color.kRed);

    // Apply the LED pattern to the data buffer
    // red.applyTo(ledBuffer);

    // Write the data to the LED strip
    // ledStrip.setData(ledBuffer);
  }

  // private void setLEDs(int r,int g, int b, int startIndex, int length){

  // }

  private void solid(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 255, 255);
    }
    ledStrip.setData(ledBuffer);
  }
}
