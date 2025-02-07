package frc.robot.subsystems.Handler;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.SubsystemManager;

public class Handler {

    private static TalonFX master = new TalonFX(HandlerConstants.HandlerMotorID);
    private static AnalogInput algaeIrInput = new AnalogInput(HandlerConstants.HandlerAnalogInputSensorID);
    private static int algaeIrValue = algaeIrInput.getValue();
    private static double power = HandlerConstants.HANDLER_POWER_STOP;

    private static DigitalInput coralIrInput = new DigitalInput(HandlerConstants.HandlerDigitalInputSensorID);
    private static boolean coralIrVal = coralIrInput.get();
    private static boolean lastCoralIrVal = coralIrInput.get();
    private static boolean isCoralIn = false;

    public static void init() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        master.getConfigurator().apply(talonFXConfigs);
    }

    public static void operate(HandlerState state) {
        updateIr(state);

        switch (state) {
            case INTAKE_ALGAE: // intake coral, deplete coral 1 - 3(level), intake algae
                power = HandlerConstants.HANDLER_POWER_INTAKE_ALGAE;
                break;
            case DEPLETE_CORAL: // deplete algae, deplete coral level 4
                power = HandlerConstants.HANDLER_POWER_DEPLETE_CORAL;
                break;
            case STOP:
                power = HandlerConstants.HANDLER_POWER_STOP;
                break;
            case DEPLETE_ALGAE:
                power = HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE;
                break;
            case HOLD_ALGAE:
                power = HandlerConstants.HANDLER_POWER_HOLD_ALGAE;
                break;
            case INTAKE_CORAL:
                power = HandlerConstants.HANDLER_POWER_INTAKE_CORAL;
                break;
        }
        
        master.setControl(new DutyCycleOut(power)); //set percent output
    }   

    private static void updateIr(HandlerState state){
        algaeIrValue = algaeIrInput.getValue();
        coralIrVal = coralIrInput.get();

        coralIrVal = !SubsystemManager.getpsJoystick().R1().getAsBoolean();
        algaeIrValue = SubsystemManager.getpsJoystick().L1().getAsBoolean() ? 0 : 2000;


        if (coralIrVal && !lastCoralIrVal) {
            isCoralIn = true;
        }else{
            isCoralIn = false;
        }

        // if (state == HandlerState.DEPLETE_CORAL) {
        //     isCoralIn = false;
        // }


        
        lastCoralIrVal = coralIrVal;
    }

    public static boolean isAlgaeIn() {
        return algaeIrValue < 1500;
        // return false;
    }

    public static boolean isCoralIn(){
        return isCoralIn;
        // return false;
    }
}

// INTAKE - intake coral, deplete coral 1 - 3(level), intake algae
// DEPLETE - deplete algae, deplete coral level 4
