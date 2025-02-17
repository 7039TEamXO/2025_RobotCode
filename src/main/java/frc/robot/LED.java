package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Handler.Handler;

public class LED {
    private static AddressableLEDBuffer buffer = new AddressableLEDBuffer(2000);
    private static Color color = Color.kOrangeRed;
    private static AddressableLED channel = new AddressableLED(1); // return to 8

    public static void init() {
        channel.setLength(buffer.getLength());
        channel.setData(buffer);
        channel.start();
    }

    public static void setLedData() {
        
        color = Color.kOrangeRed;
        if (Handler.isCoralIn() || Handler.isAlgaeIn()) {
            color = Color.kAquamarine;
        }
        if (SubsystemManager.getTxSeen()) {
            color = Color.kGreen;
        }

        for (int i = 0; i < buffer.getLength(); i++) {

            buffer.setLED(i, color);
        }
        // Maybe cache the patterns instead
        // LEDPattern pattern = LEDPattern.solid(color);
        // pattern.applyTo(buffer);
        channel.setData(buffer);
    }
}
