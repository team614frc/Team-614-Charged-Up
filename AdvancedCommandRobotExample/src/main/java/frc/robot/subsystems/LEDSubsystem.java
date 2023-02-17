package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  private final int NumLEDs = 60;

  public LEDSubsystem() {
    led = new AddressableLED(Constants.ADDRESSABLE_LED_PORT);
    ledBuffer = new AddressableLEDBuffer(NumLEDs);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setLedColorPurple() {
    for (int i = 0; i < NumLEDs; i++) {
      ledBuffer.setRGB(i, 81, 31, 192);
    }
    led.setData(ledBuffer);
  }

  public void setLedColorYellow() {
    for (int i = 0; i < NumLEDs; i++) {
      ledBuffer.setRGB(i, 251, 224, 30);
    }
    led.setData(ledBuffer);
  }
}
