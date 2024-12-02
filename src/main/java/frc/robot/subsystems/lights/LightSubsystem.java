package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {
    private final AddressableLED robotLed;
    private final AddressableLEDBuffer ledBuffer;
    private int rainbowFirstPixelHue;

    public LightSubsystem(AddressableLED robotLed, AddressableLEDBuffer buffer, int rainbowFirstPixelHue) {
        this.robotLed = new AddressableLED(ledConstants.ledPort);
        this.ledBuffer = new AddressableLEDBuffer(ledConstants.ledBufferLength);
        this.rainbowFirstPixelHue = 0;

        this.robotLed.setLength(ledBuffer.getLength());
        this.robotLed.setData(ledBuffer);
        this.robotLed.start();

    }

    public void setRainbow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }

        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;

    }

    public void periodic() {
        setRainbow();
        robotLed.setData(ledBuffer);
    }


}




