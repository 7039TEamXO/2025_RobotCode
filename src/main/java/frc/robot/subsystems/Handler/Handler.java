package frc.robot.subsystems.Handler;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;

public class Handler {

    private static TalonFX master = new TalonFX(0);
    private static AnalogInput irInput = new AnalogInput(0);
    private static int irValue = irInput.getValue();
    private static double power = 0.0;

    public static void init() {
        
    }

    public static void operate(HandlerState state) {
        switch (state) {
            case INTAKE: // intake coral, deplete coral 1 - 3(level), intake algae
                power = 0.5;
                break;
            case DEPLETE: // deplete algae, deplete coral level 4
                power = -0.5;
                break;
            case STOP:
                power = 0;
                break;
        }

        irValue = irInput.getValue();
        
        master.setControl(new DutyCycleOut(power)); //set percent output
    }   

    public static boolean isGamePieceIn() {
        return irValue < 1500;
    }
}

// INTAKE - intake coral, deplete coral 1 - 3(level), intake algae
// DEPLETE - deplete algae, deplete coral level 4
