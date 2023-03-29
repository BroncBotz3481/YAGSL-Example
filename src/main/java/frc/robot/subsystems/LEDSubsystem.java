package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led = new AddressableLED(LEDConstants.PWMPORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.LENGTH);
  private int m_rainbowFirstPixelHue;

  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {}

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }

  private void setFrontAll(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  public void setFrontHalf() {
    for (int i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      if (i < m_ledBuffer.getLength() / 2) {
        m_ledBuffer.setLED(i, Color.kBlue);
      } else {
        m_ledBuffer.setLED(i, Color.kRed);
      }
    }
  }

  public void setBackAll(Color color) {
    for (var i = m_ledBuffer.getLength() / 2; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  public void setAll(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);
  }
}