package frc.robot.subsystems.Handler;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;

public class Handler {

    private static TalonFX master = new TalonFX(HandlerConstants.HandlerMotorID);
    private static AnalogInput algaeIrInput = new AnalogInput(0);
    private static int algaeIrValue = algaeIrInput.getValue();
    private static double power = 0.0;

    public static void init() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        master.getConfigurator().apply(talonFXConfigs);
    }

    public static void operate(HandlerState state) {
        switch (state) {
            case INTAKE_ALGAE: // intake coral, deplete coral 1 - 3(level), intake algae
                power = 0.1;
                break;
            case DEPLETE_CORAL: // deplete algae, deplete coral level 4
                power = -0.1;//TODO: !!!change direction!!!
                break;
            case STOP:
                power = 0;
                break;
            case DEPLETE_ALGAE:
                power = -0.1;
                break;
            case HOLD_ALGAE:
                power = 0.1;
                break;
            case INTAKE_CORAL:
                power = 0.1;
                break;
        }

        algaeIrValue = algaeIrInput.getValue();
        
        master.setControl(new DutyCycleOut(power)); //set percent output
    }   

    public static boolean isAlgaeIn() {
        // return algaeIrValue < 1500;
        return false;
    }
}

// INTAKE - intake coral, deplete coral 1 - 3(level), intake algae
// DEPLETE - deplete algae, deplete coral level 4
