package frc.robot.subsystems.Handler;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoHub.Bank;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.subsystems.DeliveryManager;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristState;
import frc.robot.Dashboard;
import frc.robot.RobotState;


public class Handler {
    private static TalonFX master = new TalonFX(HandlerConstants.HandlerMotorID);
    
    private static AnalogInput algaeProcIrInput = new AnalogInput(HandlerConstants.HandlerAnalogProcInputSensorID);
    private static int algaeProcIrValue = algaeProcIrInput.getValue();
    private static boolean lastIsAlgaeInProcessor = false;

    private static AnalogInput algaeNetIrInput = new AnalogInput(HandlerConstants.HandlerAnalogNetInputSensorID);
    private static int algaeNetIrValue = algaeNetIrInput.getValue();
    private static boolean lastIsAlgaeInNet = false;
    
    private static double power = HandlerConstants.HANDLER_POWER_STOP;

    private static DigitalInput coralIrInput = new DigitalInput(HandlerConstants.HandlerDigitalInputSensorID);
    private static DigitalOutput coralIrOutput = new DigitalOutput(HandlerConstants.HandlerDigitalOutputSensorID);

    private static boolean coralIrVal = coralIrInput.get();
    private static boolean isCoralIn = false;
    private static boolean isAlgaeInProcessor = false;
    private static boolean isAlgaeInNet = false;
    // private static boolean lastCoralIn = false;
    // private static boolean feedCoral = false;
    private static int coralIntakeCounter = 0;
    private static int algaeDepleteCounter = 0;

    private static int algaeNetCounter = 0;

    private static double CurrenthandlerEncoderPosition = 0;
    // private static boolean isReset = false;
    private static boolean lastCoralIrVal = coralIrInput.get();
    private static boolean isFinishedDepletingAlgae = false;
    // private static boolean isAlgaeDepleteCounting = false;

    public static void init() {
        coralIrOutput.set(true);
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        master.getConfigurator().apply(talonFXConfigs);
    }

    public static void operate(HandlerState state) {
        switch (state) {
            case INTAKE_ALGAE: // intake coral, deplete coral 1 - 3 (level), intake algae
                power = HandlerConstants.HANDLER_POWER_INTAKE_ALGAE;
                break;

            case DEPLETE_CORAL: // deplete algae, deplete coral level 4
                power = HandlerConstants.HANDLER_POWER_DEPLETE_CORAL;
                break;
            
            case STOP:
                power = HandlerConstants.HANDLER_POWER_STOP;
                break;

            case DEPLETE_PROCESSOR:
                power = HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE;
                break;

            case HOLD_ALGAE:
                power = HandlerConstants.HANDLER_POWER_HOLD_ALGAE;
                break;

            case INTAKE_CORAL:
                if(SubsystemManager.getDriveBase().isAuto) {
                    power = HandlerConstants.HANDLER_POWER_INTAKE_CORAL_AUTO;
                } else{
                    power = HandlerConstants.HANDLER_POWER_INTAKE_CORAL_TELEOP;
                }
                break;

            case DEPLETE_CORAL_LEVEL0:
                power = HandlerConstants.HANDLER_POWER_DEPLETE_CORAL_LEVEL0;
                break;

            case DEPLETE_NET:
                power = HandlerConstants.HANDLER_POWER_DEPLETE_NET;
                break;

            case HOLD_NET:
                power = HandlerConstants.HANDLER_POWER_HOLD_NET;
                break;
            case INTAKE_NET:
            power = HandlerConstants.HANDLER_POWER_INTAKE_NET;
                break;
        }

        if((SubsystemManager.getElevatorState() == ElevatorState.INTAKE_CORAL ||
            SubsystemManager.getElevatorState() == ElevatorState.LEVEL0 ||
            SubsystemManager.getElevatorState() == ElevatorState.LEVEL1 || 
            SubsystemManager.getElevatorState() == ElevatorState.LEVEL2 || 
            SubsystemManager.getElevatorState() == ElevatorState.BASE) && SubsystemManager.getIsMoveCoral()) {
            master.setControl(new DutyCycleOut(HandlerConstants.HANDLER_POWER_PUSH_BACK_CORAL));
        } else if (SubsystemManager.getElevatorState() == ElevatorState.LEVEL3 && SubsystemManager.getIsMoveCoral()){
            master.setControl(new DutyCycleOut(-HandlerConstants.HANDLER_POWER_PUSH_BACK_CORAL));
        }
        else if (state != HandlerState.DEPLETE_PROCESSOR && SubsystemManager.getElevatorState() == ElevatorState.LEVEL3 && 
            Math.abs(getHandlerMotorDistance() - CurrenthandlerEncoderPosition) <= HandlerConstants.LEVEL4_CORAL_PUSH_DISTANCE){
            // System.out.println(state);
            master.setControl(new DutyCycleOut(-HandlerConstants.HANDLER_POWER_PUSH_BACK_CORAL));
        } 
        else {
            master.setControl(new DutyCycleOut(power + Dashboard.addValueToHandler())); //set percent output
        }

        // System.out.println("stator " + master.getStatorCurrent().getValueAsDouble());
    }   


    public static void updateHandlerIr(RobotState state, ElevatorState elevatorState, HandlerState handlerState) { // boolean isReset
        algaeProcIrValue = algaeProcIrInput.getValue();
        algaeNetIrValue = algaeNetIrInput.getValue();
        coralIrVal = getCoralIr();

        // CORAL
        
        if (coralIrVal) {
            coralIntakeCounter++;
            isCoralIn = false;
        }
        else if(!lastCoralIrVal) {
            coralIntakeCounter = 0;
        }

        if (!coralIrVal && state != RobotState.DEPLETE && coralIntakeCounter > HandlerConstants.CORAL_IN_DEBOUNCE_COUNTER){ //16
            isCoralIn = true;
            CurrenthandlerEncoderPosition = getHandlerMotorDistance();
        }

        if (state == RobotState.DEPLETE) {
            isCoralIn = false;
            coralIntakeCounter = 0;
        }

        //ALGAE

        if(handlerState == HandlerState.DEPLETE_PROCESSOR || handlerState == HandlerState.DEPLETE_NET) {
            algaeDepleteCounter ++;
        } else {
            algaeDepleteCounter = 0;
        }

        //ALGAE PROCESSOR

        isAlgaeInProcessor = ((algaeProcIrValue > HandlerConstants.ALGAE_PROC_IR_IN_VALUE && state == RobotState.INTAKE && Wrist.isWristAtSetPoint() &&
         (elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR || elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR ))
         || lastIsAlgaeInProcessor) && algaeDepleteCounter < 35 && !isCoralIn;

        //ALGAE NET

        // System.out.println(master.getStatorCurrent().getValueAsDouble());

        // CURRENT IMPLEMENTATION (TEMPORARY)

        // isAlgaeInNet = ((master.getStatorCurrent().getValueAsDouble() > 40 && state == RobotState.INTAKE &&
        // (elevatorState == ElevatorState.ALGAE_HIGH_NET || elevatorState == ElevatorState.ALGAE_LOW_NET)) || lastIsAlgaeInNet) 
        // && algaeDepleteCounter < 35 && !isCoralIn && !SubsystemManager.getpsJoystick().getHID().getCrossButton();

        // SENSOR IMPLEMENTATION
        // if(algaeNetIrValue > HandlerConstants.ALGAE_NET_IR_IN_VALUE){
        //     algaeNetCounter ++;
        // }else{
        //     algaeNetCounter = 0;
        // }

        isAlgaeInNet = (((algaeNetIrValue > HandlerConstants.ALGAE_NET_IR_IN_VALUE && Wrist.isWristAtSetPoint()) && state == RobotState.INTAKE && 
        (elevatorState == ElevatorState.ALGAE_HIGH_NET || elevatorState == ElevatorState.ALGAE_LOW_NET)) || lastIsAlgaeInNet) 
        && algaeDepleteCounter < 35 && !isCoralIn && !SubsystemManager.getpsJoystick().getHID().getCrossButton();

        isFinishedDepletingAlgae = (!isAlgaeInProcessor && lastIsAlgaeInProcessor) || (!isAlgaeInNet && lastIsAlgaeInNet);

        lastCoralIrVal = coralIrVal;
        // lastCoralIn = isCoralIn;
        lastIsAlgaeInProcessor = isAlgaeInProcessor;
        lastIsAlgaeInNet = isAlgaeInNet;
    }

    public static int getCoralIntakeCounter() {
        return coralIntakeCounter;
    }

    public static boolean isAlgaeInProcessor() {
        return isAlgaeInProcessor;
    }

    public static boolean isAlgaeInNet() {
        return isAlgaeInNet;
    }

    public static boolean isCoralIn() {
        return isCoralIn;
    }
    
    // public static boolean getFeedCoral() {
    //     return feedCoral;
    // }

    public static boolean getCoralIr() {
        return coralIrInput.get();
    }

    public static int getAlgaeProcIrValue() {
        return algaeProcIrInput.getValue();
    }

    public static int getAlgaeNetIrValue() {
        return algaeNetIrInput.getValue();
    }

    public static double getHandlerMotorDistance() {
        return master.getPosition().getValueAsDouble();
    }

    public static boolean isFinishedDepletingAlgae() {
        return isFinishedDepletingAlgae;
    }
}

// INTAKE - intake coral, deplete coral 1 - 3 (level), intake algae
// DEPLETE - deplete algae, deplete coral level 4