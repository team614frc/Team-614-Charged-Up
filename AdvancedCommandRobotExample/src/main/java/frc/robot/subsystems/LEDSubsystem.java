package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  private final int NumLEDs = 180;

  private int rainbowInitialHue = 0;

  public LEDSubsystem() {
    led = new AddressableLED(Constants.ADDRESSABLE_LED_PORT);
    ledBuffer = new AddressableLEDBuffer(NumLEDs);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setLedColorOrange() {
    for (int i = 0; i < NumLEDs; i++) {
      ledBuffer.setRGB(i, 255, 17, 0);
    }
    led.setData(ledBuffer);
  }

  public void setLedColorGreen() {
    for (int i = 0; i < NumLEDs; i++) {
      ledBuffer.setRGB(i, 60, 255, 0);
    }
    led.setData(ledBuffer);
  }

  public void setLedColorPurple() {
    for (int i = 0; i < NumLEDs; i++) {
      ledBuffer.setRGB(i, 50, 0, 50);
    }
    led.setData(ledBuffer);
  }

  public void setLedColorYellow() {
    for (int i = 0; i < NumLEDs; i++) {
      ledBuffer.setRGB(i, 255, 95, 0);
    }
    led.setData(ledBuffer);
  }

  public void setLedColorRainbow() {
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowInitialHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowInitialHue += 3;
    // Check bounds
    rainbowInitialHue %= 180;
  }
}
