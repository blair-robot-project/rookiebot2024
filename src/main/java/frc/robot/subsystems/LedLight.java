package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.allConstants.ledConstants;
import frc.robot.allConstants.ledConstants.*;

public class LedLight extends SubsystemBase {
    private AddressableLED robotLed;
    private AddressableLEDBuffer Ledbuffer;
    private int rainbowFirstPixelHue;

    public LedLight(AddressableLED robotLed, AddressableLEDBuffer buffer,int rainbowFirstPixelHue) {
        this.robotLed = new AddressableLED(ledConstants.ledPort);
        this.Ledbuffer = new AddressableLEDBuffer(ledConstants.ledBuffer);
        this.rainbowFirstPixelHue=0;

        this.robotLed.setLength(Ledbuffer.getLength());
        this.robotLed.setData(Ledbuffer);
        this.robotLed.start();

    }
    public void setRainbow(){
        for (int i=0; i<Ledbuffer.getLength();i++ ){
            final var hue = (rainbowFirstPixelHue+(i *180/Ledbuffer.getLength()))%180;
            Ledbuffer.setHSV(i,hue,255,128);
        }
        rainbowFirstPixelHue+=3;
        rainbowFirstPixelHue%=180;

    }
    public void periodic(){
        setRainbow();
        robotLed.setData(Ledbuffer);
    }


}




