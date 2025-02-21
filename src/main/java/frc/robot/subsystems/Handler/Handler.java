package frc.robot.subsystems.Handler;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.subsystems.DeliveryManager;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class Handler {
    private static TalonFX master = new TalonFX(HandlerConstants.HandlerMotorID);
    private static AnalogInput algaeIrInput = new AnalogInput(HandlerConstants.HandlerAnalogInputSensorID);
    private static int algaeIrValue = algaeIrInput.getValue();
    private static int lastAlgaeIrValue = algaeIrInput.getValue();
    private static double power = HandlerConstants.HANDLER_POWER_STOP;

    private static DigitalInput coralIrInput = new DigitalInput(HandlerConstants.HandlerDigitalInputSensorID);
    private static DigitalOutput coralIrOutput = new DigitalOutput(HandlerConstants.HandlerDigitalOutPutSensorID);
    private static boolean coralIrVal = coralIrInput.get();
    private static boolean isCoralIn = false;
    private static boolean lastCoralIn = false;
    private static boolean feedCoral = false;
    private static boolean startCountingCoral = false;
    private static boolean startCountingAlgae = false;
    private static int countingAlgae = 0;
    private static int countingCoral = 0;
    private static int counter = 0;
    private static boolean isReset = false;
    private static boolean lastCoralIrVal = coralIrInput.get();

    public static void init() {
        coralIrOutput.set(true);
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        master.getConfigurator().apply(talonFXConfigs);
    }

    public static void operate(HandlerState state) {
        // updateHandlerIr();
        switch (state) {
            case INTAKE_ALGAE: // intake coral, deplete coral 1 - 3 (level), intake algae
                // isReset = true;
                power = HandlerConstants.HANDLER_POWER_INTAKE_ALGAE;
                break;

            case DEPLETE_CORAL: // deplete algae, deplete coral level 4
                // isReset = true;
                power = HandlerConstants.HANDLER_POWER_DEPLETE_CORAL;
                break;
            
            case STOP:
                power = HandlerConstants.HANDLER_POWER_STOP;
                break;

            case DEPLETE_ALGAE:
                // isReset = true;
                power = HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE;
                break;

            case HOLD_ALGAE:
                power = HandlerConstants.HANDLER_POWER_HOLD_ALGAE;
                break;

            case INTAKE_CORAL:
                // isReset = false;
                power = HandlerConstants.HANDLER_POWER_INTAKE_CORAL;
                break;

            case DEPLETE_CORAL_LEVEL0:
                // isReset = true;
                power = HandlerConstants.HANDLER_POWER_DEPLETE_CORAL_LEVEL0;
                break;
                
            case PUSH_BACK_CORAL:
                power = HandlerConstants.HANDLER_POWER_PUSH_BACK_CORAL;
                break;
        }

        //System.out.println(algaeIrInput.getValue());
        master.setControl(new DutyCycleOut(power)); //set percent output
        // System.out.println(power);
        // System.out.println(coralIrInput.get());
    }   


    public static void updateHandlerIr(){ //boolean isReset
        algaeIrValue = algaeIrInput.getValue();
        coralIrVal = getCoralIr();
        // System.out.println("UpdHIR [!]");
        
        // System.out.println("[IsReset] " + isReset);
        // if (!coralIrVal && lastCoralIrVal) {
        //         isCoralIn = true;
        // }
        if (coralIrVal) {
            counter++;
        } else if(!lastCoralIrVal) {
            counter = 0;
        }
        // if (isReset) {
        //     isCoralIn = false;
        //     counter = 0;

        // }

        if (!coralIrVal && (power != HandlerConstants.HANDLER_POWER_DEPLETE_CORAL &&
                power != HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE && 
                    power != HandlerConstants.HANDLER_POWER_DEPLETE_CORAL_LEVEL0) && counter > 16) {
            isCoralIn = true;
        }
        if (power == HandlerConstants.HANDLER_POWER_DEPLETE_CORAL ||
                power == HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE || 
                    power == HandlerConstants.HANDLER_POWER_DEPLETE_CORAL_LEVEL0 ||
                        power == HandlerConstants.HANDLER_POWER_INTAKE_ALGAE) {
            isCoralIn = false;
            counter = 0;
        }
        if (power == HandlerConstants.HANDLER_POWER_DEPLETE_CORAL ||
                power == HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE || 
                    power == HandlerConstants.HANDLER_POWER_DEPLETE_CORAL_LEVEL0) {
                        startCountingCoral = true;
                        }
        if (power == HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE && SubsystemManager.getElevatorState() != ElevatorState.LEVEL3) {
            startCountingAlgae = true;

        }
        if (startCountingCoral) {
            countingCoral ++;
        }
        if (startCountingAlgae) {
            power = HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE;
            countingAlgae ++;
        }

        // System.out.println(counter);
        // if ((power == HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE) && lastCoralIn) {
        //     isCoralIn = false;
        // }
        if ((countingCoral > 15)) {
            startCountingCoral = false;
            countingCoral = 0;
        }
        if(countingAlgae > 20) {
            startCountingAlgae = false;
            countingAlgae = 0;
        }

        System.out.println("coral--" + countingCoral);
        System.out.println("coral--" +startCountingCoral);
        System.out.println("Algae++" +countingAlgae);
        System.out.println("Algae++" +startCountingAlgae);
        
        lastCoralIrVal = coralIrVal;
        lastCoralIn = isCoralIn;
        lastAlgaeIrValue = algaeIrValue;
    }

    public static int getCounter() {
        return counter;
    }

    public static boolean getReset() {
        return isReset;
    }
    public static boolean isAlgaeIn() {
        if (isCoralIn) {
            return false;
        } else{
            return ((countingAlgae < 20 && countingAlgae > 0) && !startCountingCoral) || algaeIrValue > HandlerConstants.ALGAE_IR_IN_VALUE; // return to this moment
        }
    }

    public static boolean isCoralIn() {
        return isCoralIn;
    }
    
    public static boolean getFeedCoral() {
        return feedCoral;
    }
    public static boolean getCoralIr() {
        return coralIrInput.get();
    }


    public static int getAlgaeIrValue(){
        return algaeIrValue;
    }
}

// INTAKE - intake coral, deplete coral 1 - 3 (level), intake algae
// DEPLETE - deplete algae, deplete coral level 4