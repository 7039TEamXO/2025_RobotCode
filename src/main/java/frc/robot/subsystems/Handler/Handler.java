package frc.robot.subsystems.Handler;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.IO.HandlerIO;
import frc.robot.subsystems.IO.HandlerIO.HandlerIOInputs;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.RobotState;


public class Handler {
    private static HandlerIO io;
    private static HandlerIOInputs inputs = new HandlerIOInputs();
    
    private static int algaeProcIRValue = -1;
    private static boolean lastIsAlgaeInProcessor = false;

    private static int algaeNetIRValue = -1;
    private static boolean lastIsAlgaeInNet = false;
    
    private static double power = HandlerConstants.HANDLER_POWER_STOP;

    private static boolean coralIRValue = false;
    private static boolean isCoralIn = false;
    private static boolean isAlgaeInProcessor = false;
    private static boolean isAlgaeInNet = false;

    // private static boolean lastCoralIn = false;
    // private static boolean feedCoral = false;
    private static int coralIntakeCounter = 0;
    private static int algaeDepleteCounter = 0;

    private static double CurrentHandlerEncoderPosition = 0;
    // private static boolean isReset = false;
    private static boolean lastCoralIRValue = false;
    private static boolean isFinishedDepletingAlgae = false;
    // private static boolean isAlgaeDepleteCounting = false;

    public static void init(HandlerIO _io) {        
        io = _io;

        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        io.applyTalonFXConfig(talonFXConfigs);

        io.updateInputs(inputs);
        Logger.processInputs("Handler", inputs);
    }

    public static void operate(HandlerState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Handler", inputs);

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
                if(Robot.isAuto()) {
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
            io.setMotionMagic(new DutyCycleOut(HandlerConstants.HANDLER_POWER_PUSH_BACK_CORAL));
        } else if (SubsystemManager.getElevatorState() == ElevatorState.LEVEL3 && SubsystemManager.getIsMoveCoral()){
            io.setMotionMagic(new DutyCycleOut(-HandlerConstants.HANDLER_POWER_PUSH_BACK_CORAL));
        }
        else if (state != HandlerState.DEPLETE_PROCESSOR && SubsystemManager.getElevatorState() == ElevatorState.LEVEL3 && 
             Math.abs(getHandlerMotorDistance() - CurrentHandlerEncoderPosition) <= HandlerTuning.LEVEL4_CORAL_PUSH_DISTANCE_get()) {
            io.setMotionMagic(new DutyCycleOut(-HandlerConstants.HANDLER_POWER_PUSH_BACK_CORAL));
        } 
        else {
            io.setMotionMagic(new DutyCycleOut(power + Dashboard.addValueToHandler())); //set percent output
        }
    }

    public static void updateHandlerIR(RobotState state, ElevatorState elevatorState, HandlerState handlerState) { // boolean isReset
        algaeProcIRValue = getAlgaeProcessorIR();
        algaeNetIRValue = getAlgaeNetIR();
        coralIRValue = getCoralIR();

        // CORAL
        
        if (coralIRValue) {
            coralIntakeCounter++;
            isCoralIn = false;
        }
        else if(!lastCoralIRValue) {
            coralIntakeCounter = 0;
        }

        if (!coralIRValue && state != RobotState.DEPLETE && coralIntakeCounter > HandlerTuning.CORAL_IN_DEBOUNCE_COUNTER_get()) {
            isCoralIn = true;
            CurrentHandlerEncoderPosition = getHandlerMotorDistance();
        }

        if (state == RobotState.DEPLETE) {
            isCoralIn = false;
            coralIntakeCounter = 0;
        }

        // ALGAE

        if(handlerState == HandlerState.DEPLETE_PROCESSOR || handlerState == HandlerState.DEPLETE_NET) {
            algaeDepleteCounter++;
        } else {
            algaeDepleteCounter = 0;
        }

        // ALGAE PROCESSOR

        if ((algaeProcIRValue > HandlerTuning.ALGAE_PROC_IR_IN_VALUE_get() || Math.abs(inputs.currentAmps) > 80) && state == RobotState.INTAKE 
            && Wrist.isWristAtSetPoint() && (elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR || elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR) && !isCoralIn) {
            isAlgaeInProcessor = true;
        } else if (algaeDepleteCounter > 35) {
            isAlgaeInProcessor = false;
        }

        // ALGAE NET

        if((algaeNetIRValue > HandlerTuning.ALGAE_NET_IR_IN_VALUE_get() || Math.abs(inputs.currentAmps) > 80) && Wrist.isWristAtSetPoint()
            && state == RobotState.INTAKE && (elevatorState == ElevatorState.ALGAE_HIGH_NET || elevatorState == ElevatorState.ALGAE_LOW_NET) && !isCoralIn) {
            isAlgaeInNet = true;
        } else if(algaeDepleteCounter > 35 || SubsystemManager.getPSJoystick().getHID().getCrossButton()) {
            isAlgaeInNet = false;
        }

        isFinishedDepletingAlgae = (!isAlgaeInProcessor && lastIsAlgaeInProcessor) || (!isAlgaeInNet && lastIsAlgaeInNet);

        if(Dashboard.getAcceptCoralChanges()) isCoralIn = Dashboard.getChosenIsCoralIn();
        if(Dashboard.getAcceptAlgaeChanges()) {
            isAlgaeInProcessor = Dashboard.getChosenIsAlgaeInProcessor();
            isAlgaeInNet = Dashboard.getChosenIsAlgaeInNet();
        }

        lastCoralIRValue = coralIRValue;
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

    public static boolean getCoralIR() {
        return inputs.coralIR;
    }

    public static int getAlgaeProcessorIR() {
        return inputs.algaeProcessorIR;
    }

    public static int getAlgaeNetIR() {
        return inputs.algaeNetIR;
    }

    public static double getHandlerMotorDistance() {
        return inputs.position;
    }

    public static boolean isFinishedDepletingAlgae() {
        return isFinishedDepletingAlgae;
    }

    public static void simulationPeriodic() {
        io.simulationPeriodic();
    }
}

// INTAKE - intake coral, deplete coral 1 - 3 (level), intake algae
// DEPLETE - deplete algae, deplete coral level 4