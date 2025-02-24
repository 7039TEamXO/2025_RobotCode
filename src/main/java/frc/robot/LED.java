package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Handler.Handler;

public class LED {
    private static AddressableLEDBuffer buffer = new AddressableLEDBuffer(2000);
    private static Color color = Color.kOrangeRed;
    private static AddressableLED midChannel = new AddressableLED(1); // return to 8
    // private static AddressableLED leftChannel = new AddressableLED(3);
    // private static AddressableLED rightChannel = new AddressableLED(2);

    public static void init() {
        midChannel.setLength(buffer.getLength());
        midChannel.setData(buffer);
        midChannel.start();
        // leftChannel.setLength(buffer.getLength());
        // leftChannel.setData(buffer);
        // leftChannel.start();
        // rightChannel.setLength(buffer.getLength());
        // rightChannel.setData(buffer);
        // rightChannel.start();
    }

    public static void setLedData() {
        color = Color.kOrangeRed;
        if (Handler.isCoralIn() || Handler.isAlgaeIn()) {
            color = Color.kDeepPink;
        }
        if (SubsystemManager.getTxSeen()) {
            color = Color.kGreen;
        }

        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        
        // LEDPattern pattern = LEDPattern.solid(color);
        // pattern.applyTo(buffer);
        
        midChannel.setData(buffer);
        // rightChannel.setData(buffer);
        // leftChannel.setData(buffer);
    }
}
