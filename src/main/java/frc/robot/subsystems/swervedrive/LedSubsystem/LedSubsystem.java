package frc.robot.subsystems.swervedrive.LedSubsystem;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private int rainbowFirstPixelHue = 0;

    public LedSubsystem(int pwmPort, int ledLength) {
        led = new AddressableLED(pwmPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void setColor(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    public void setMovingRainbow() {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
          // Each LED gets a hue that shifts over time
          int hue = (rainbowFirstPixelHue + (i * 10)) % 180; // Spread the hue changes across the strip
          ledBuffer.setHSV(i, hue, 255, 128); // Full saturation, medium brightness
      }
  
      // Shift the entire pattern forward for animation
      rainbowFirstPixelHue += 3;  
      rainbowFirstPixelHue %= 180;
  
      led.setData(ledBuffer);
  }
    @Override
    public void periodic() {
        // This could update animations automatically, like a rainbow effect.
        
    }
}
