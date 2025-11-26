package frc.robot.subsystems.Handler;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.IO.HandlerIO;
import frc.robot.subsystems.IO.HandlerIO.HandlerIOInputs;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
public class Handler implements Subsystem {
    private HandlerIO io;
    private HandlerIOInputs inputs = new HandlerIOInputs();
    
    private int algaeProcIRValue = -1;
    private boolean lastIsAlgaeInProcessor = false;

    private int algaeNetIRValue = -1;
    private boolean lastIsAlgaeInNet = false;
    
    private double power = HandlerConstants.HANDLER_POWER_STOP;

    private boolean coralIRValue = false;
    private boolean isCoralIn = false;
    private boolean isAlgaeInProcessor = false;
    private boolean isAlgaeInNet = false;

    // private boolean lastCoralIn = false;
    // private boolean feedCoral = false;
    private int coralIntakeCounter = 0;
    private int algaeDepleteCounter = 0;

    private double CurrentHandlerEncoderPosition = 0;
    // private boolean isReset = false;
    private boolean lastCoralIRValue = false;
    private boolean isFinishedDepletingAlgae = false;
    // private boolean isAlgaeDepleteCounting = false;
 
    public Handler(HandlerIO _io) {        
        io = _io;

        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        io.applyTalonFXConfig(talonFXConfigs);

        io.updateInputs(inputs);
        Logger.processInputs("Handler", inputs);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(HandlerConstants.SysIdQuasistatic), 
                Volts.of(HandlerConstants.SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> io.setVoltage(v), null, this));
    }

    public void operate(HandlerState state) {
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
                    power = HandlerConstants.HANDLER_POWER_INTAKE_CORAL_TELEOP;
                } else {
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

    public void updateHandlerIR(RobotState state, ElevatorState elevatorState, HandlerState handlerState) { // boolean isReset
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
        /*
         * (coralIntakeCounter > HandlerTuning.CORAL_IN_DEBOUNCE_COUNTER_get() && !Robot.isAuto()) ||
            (coralIntakeCounter > HandlerTuning.CORAL_IN_DEBOUNCE_COUNTER_AUTO_get() && Robot.isAuto())
         */
        if (!coralIRValue && state != RobotState.DEPLETE && 
            coralIntakeCounter > HandlerTuning.CORAL_IN_DEBOUNCE_COUNTER_get()) {
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
        // || Math.abs(inputs.currentAmps) > 80
        if ((algaeProcIRValue > HandlerTuning.ALGAE_PROC_IR_IN_VALUE_get()) && state == RobotState.INTAKE 
            && RobotContainer.wrist.isWristAtSetPoint() && (elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR || elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR) && !isCoralIn) {
            isAlgaeInProcessor = true;
        } else if (algaeDepleteCounter > 35) {
            isAlgaeInProcessor = false;
        }

        // ALGAE NET
        // || Math.abs(inputs.currentAmps) > 80
        if((algaeNetIRValue > HandlerTuning.ALGAE_NET_IR_IN_VALUE_get() && algaeNetIRValue < HandlerTuning.ALGAE_NET_IR_NOT_IN_VALUE_get()) && RobotContainer.wrist.isWristAtSetPoint()
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

    public int getCoralIntakeCounter() {
        return coralIntakeCounter;
    }

    public boolean isAlgaeInProcessor() {
        return isAlgaeInProcessor;
    }

    public boolean isAlgaeInNet() {
        return isAlgaeInNet;
    }

    public boolean isCoralIn() {
        return isCoralIn;
    }
    
    // public boolean getFeedCoral() {
    //     return feedCoral;
    // }

    public boolean getCoralIR() {
        return inputs.coralIR;
    }

    public int getAlgaeProcessorIR() {
        return inputs.algaeProcessorIR;
    }

    public int getAlgaeNetIR() {
        return inputs.algaeNetIR;
    }

    public double getHandlerMotorDistance() {
        return inputs.position;
    }

    public boolean isFinishedDepletingAlgae() {
        return isFinishedDepletingAlgae;
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    // SysId routines //

    private SysIdRoutine sysIdRoutine;    

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}

// INTAKE - intake coral, deplete coral 1 - 3 (level), intake algae
// DEPLETE - deplete algae, deplete coral level 4