package frc.robot.subsystems.Handler;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DeliveryManager;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristState;
import frc.robot.RobotState;


public class Handler {
    private static TalonFX master = new TalonFX(HandlerConstants.HandlerMotorID);
    private static AnalogInput algaeIrInput = new AnalogInput(HandlerConstants.HandlerAnalogInputSensorID);
    private static int algaeIrValue = algaeIrInput.getValue();
    private static boolean lastIsAlgaeIn = false;
    private static double power = HandlerConstants.HANDLER_POWER_STOP;

    private static DigitalInput coralIrInput = new DigitalInput(HandlerConstants.HandlerDigitalInputSensorID);
    private static DigitalOutput coralIrOutput = new DigitalOutput(HandlerConstants.HandlerDigitalOutPutSensorID);
    private static boolean coralIrVal = coralIrInput.get();
    private static boolean isCoralIn = false;
    private static boolean isAlgaeIn = false;
    private static boolean lastCoralIn = false;
    private static boolean feedCoral = false;
    private static int coralIntakeCounter = 0;
    private static int algaeDepleteCounter = 0;
    private static boolean isReset = false;
    private static boolean lastCoralIrVal = coralIrInput.get();
    private static boolean isFinishedDepletingAlgae = false;
    private static boolean isAlgaeDepleteCounting = false;

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

            case DEPLETE_ALGAE:
                power = HandlerConstants.HANDLER_POWER_DEPLETE_ALGAE;
                break;

            case HOLD_ALGAE:
                power = HandlerConstants.HANDLER_POWER_HOLD_ALGAE;
                break;

            

            case INTAKE_CORAL:
            if(SubsystemManager.getDriveBase().isAuto){
                power = HandlerConstants.HANDLER_POWER_INTAKE_CORAL_AUTO;
            }else{
                power = HandlerConstants.HANDLER_POWER_INTAKE_CORAL_TELEOP;
            }
                break;

            case DEPLETE_CORAL_LEVEL0:
                power = HandlerConstants.HANDLER_POWER_DEPLETE_CORAL_LEVEL0;
                break;
                 
            case SHOOT_ALGAE:
            if (Elevator.getCurrentPosition() > 10){
                power = HandlerConstants.SHOOT_ALGAE;
            }
            else {
                power = HandlerConstants.HOLD_ALGAE_FOR_SHOOT;
            }
                break;

        }

        if((SubsystemManager.getElevatorState() == ElevatorState.INTAKE_CORAL ||
                SubsystemManager.getElevatorState() == ElevatorState.LEVEL0 ||
                    SubsystemManager.getElevatorState() == ElevatorState.LEVEL1 || 
                        SubsystemManager.getElevatorState() == ElevatorState.LEVEL2 || 
                            SubsystemManager.getElevatorState() == ElevatorState.BASE) && SubsystemManager.getIsMooveCoral()) {
                                master.setControl(new DutyCycleOut(HandlerConstants.HANDLER_POWER_PUSH_BACK_CORAL));
                            } else if (SubsystemManager.getElevatorState() == ElevatorState.LEVEL3 && SubsystemManager.getIsMooveCoral())
                            master.setControl(new DutyCycleOut(-HandlerConstants.HANDLER_POWER_PUSH_BACK_CORAL));
                            else {
                                master.setControl(new DutyCycleOut(power)); //set percent output
                            }

        
    }   


    public static void updateHandlerIr(RobotState state, ElevatorState elevatorState) { // boolean isReset
        algaeIrValue = algaeIrInput.getValue();
        coralIrVal = getCoralIr();
        
        if (coralIrVal){
            coralIntakeCounter++;
            isCoralIn = false;
        }
        else if(!lastCoralIrVal){
            coralIntakeCounter = 0;
        }

        
        if (!coralIrVal && state != RobotState.DEPLETE && coralIntakeCounter > HandlerConstants.CORAL_IN_DEBOUCE_COUNTER){ //16
            isCoralIn = true;
        }

        if (state == RobotState.DEPLETE) {
            isCoralIn = false;
            coralIntakeCounter = 0;
        }
        // System.out.println(coralIntakeCounter);

        isAlgaeDepleteCounting = (state == RobotState.DEPLETE) ? true : isAlgaeDepleteCounting;

        if(!isAlgaeIn) isAlgaeDepleteCounting = false;

        if (isAlgaeDepleteCounting) 
            algaeDepleteCounter++;
        else algaeDepleteCounter = 0;

        if (!isCoralIn && (elevatorState == ElevatorState.ALGAE_HIGH || elevatorState == ElevatorState.ALGAE_LOW
        || elevatorState == ElevatorState.ALGAE_HIGH_IN || elevatorState == ElevatorState.ALGAE_LOW_IN 
        || elevatorState == ElevatorState.BASE)){
            isAlgaeIn = (algaeIrValue > HandlerConstants.ALGAE_IR_IN_VALUE || (lastIsAlgaeIn && algaeDepleteCounter < 20)) && elevatorState != ElevatorState.BASE
            ? true : elevatorState == ElevatorState.BASE && (lastIsAlgaeIn && algaeDepleteCounter < 20) ? isAlgaeIn : false; // if algae is detected isAlgaeIn will be true until deplete
        }else{ isAlgaeIn = false;}

        isFinishedDepletingAlgae = !isAlgaeIn && lastIsAlgaeIn;

        lastCoralIrVal = coralIrVal;
        lastCoralIn = isCoralIn;
        lastIsAlgaeIn = isAlgaeIn;
    }

    public static int getCoralIntakeCounter() {
        return coralIntakeCounter;
    }

    public static boolean getReset() {
        return isReset;
    }

    public static boolean isAlgaeIn() {
        return isAlgaeIn;
        
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
        return algaeIrInput.getValue();
        // return algaeIrValue;

    }

    public static boolean isFinishedDepletingAlgae(){
        return isFinishedDepletingAlgae;
    }
}

// INTAKE - intake coral, deplete coral 1 - 3 (level), intake algae
// DEPLETE - deplete algae, deplete coral level 4