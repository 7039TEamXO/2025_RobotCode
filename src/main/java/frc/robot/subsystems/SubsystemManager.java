package frc.robot.subsystems;

import java.io.File;
import java.lang.annotation.ElementType;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.LED;
import frc.robot.Limelight;
import frc.robot.RobotState;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Handler.HandlerState;
import frc.robot.subsystems.Tray.Tray;
import frc.robot.subsystems.Tray.TrayState;
import frc.robot.subsystems.Handler.Handler;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristState;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SubsystemManager {
    private static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));

    private static final CommandPS4Controller ps4Joystick = new CommandPS4Controller(0);
    private static final PS4Controller psController_HID = ps4Joystick.getHID();

    private static boolean isLocked = false;

    private static RobotState state = RobotState.TRAVEL;
    private static RobotState lastState;

    private static ElevatorState elevatorState = ElevatorState.BASE;
    private static ElevatorState lastElevatorState = ElevatorState.BASE;

    private static int autoElevatorCounter = 0;
    private static boolean isAutoElevatorCounting = false;
    
    private static HandlerState handlerState = HandlerState.STOP;

    private static ClimbState climbState = ClimbState.STOP;
    private static TrayState trayState = TrayState.BASE;

    private static boolean isTxSeen = false;

    private static boolean isResetWrist = false;

    private static boolean isMooveCoral = false;


    public static Command travelCommand = Commands.run(() -> operateAuto(RobotState.TRAVEL, null));
    public static Command intakeCoralCommand = Commands.run(() -> operateAuto(RobotState.INTAKE, ElevatorState.BASE));
    public static Command intakeAlgaeLowCommand = Commands.run(() -> operateAuto(RobotState.INTAKE, ElevatorState.ALGAE_LOW));
    public static Command intakeAlgaeHighCommand = Commands.run(() -> operateAuto(RobotState.INTAKE, ElevatorState.ALGAE_HIGH));
    public static Command baseCommand = Commands.run(() -> operateAuto(null, ElevatorState.BASE));
    public static Command level0Command = Commands.run(() -> operateAuto(null, ElevatorState.LEVEL0));
    public static Command level1Command = Commands.run(() -> operateAuto(null, ElevatorState.LEVEL1));
    public static Command level2Command = Commands.run(() -> operateAuto(null, ElevatorState.LEVEL2));
    public static Command level3Command = Commands.run(() -> operateAuto(null, ElevatorState.LEVEL3));
    public static Command depleteCommand = Commands.run(() -> operateAuto(RobotState.DEPLETE, null));
    // public static Command alignCommand = Commands.run(() -> getDriveBase().alignByLimelight(() -> 0));
    // public static Command selectRightLimelight = Commands.runOnce(() -> Limelight.setPipeline(0));
    // public static Command selectLeftLimelight = Commands.runOnce(() -> Limelight.setPipeline(1));
    // public static Command selectMidLimelight = Commands.runOnce(() -> Limelight.setPipeline(2));

    public static void init() {
        state = RobotState.TRAVEL;
        lastState = state;
        DeliveryManager.init();
        Handler.init();
        Climb.init();
        Tray.init();
    }

    public static void operate(boolean onAuto) { 
        isResetWrist = psController_HID.getTouchpadButton();
        // System.out.println();
        // if (psController_HID.getTouchpadButton()) {
        //     DeliveryManager.resetWrist();

        // }
        
        if (!onAuto) {
            state = psController_HID.getPOV(0) == 0 ? RobotState.TRAVEL : 
            state == RobotState.CLIMB ? RobotState.CLIMB :
            psController_HID.getL2Button() ? RobotState.DEPLETE :
            psController_HID.getR2Button() ? RobotState.INTAKE :
            psController_HID.getShareButton() ? RobotState.CLIMB :
            psController_HID.getPOV(0) == 90 ? RobotState.INTAKE :
            psController_HID.getPOV(0) == 270 ? RobotState.INTAKE :
            psController_HID.getCrossButton() && !Handler.isAlgaeIn() ? 
                RobotState.INTAKE : lastState;

            elevatorState = state == RobotState.CLIMB ? ElevatorState.BASE :
            psController_HID.getCrossButton() ? ElevatorState.BASE : // BASE
            psController_HID.getSquareButton() ? ElevatorState.LEVEL1 :
            psController_HID.getTriangleButton() ? ElevatorState.LEVEL3 :
            psController_HID.getCircleButton() ? ElevatorState.LEVEL2 :
            psController_HID.getPOV(0) == 90 ? ElevatorState.ALGAE_HIGH : // right
            psController_HID.getPOV(0) == 180 ? ElevatorState.LEVEL0 : // down
            psController_HID.getPOV(0) == 270 ? ElevatorState.ALGAE_LOW : // left
            lastElevatorState;
        }
        Handler.updateHandlerIr(state, elevatorState);
        switch (state) {
            case TRAVEL:
                if(Handler.isAlgaeIn() && (elevatorState == ElevatorState.ALGAE_LOW || 
                                            elevatorState == ElevatorState.ALGAE_HIGH ||
                                              elevatorState == ElevatorState.BASE))
                    handlerState = HandlerState.HOLD_ALGAE;
                else
                    handlerState = HandlerState.STOP;
                
                if ((lastElevatorState == ElevatorState.INTAKE_CORAL && Handler.isCoralIn()) && elevatorState != ElevatorState.LEVEL0)
                    elevatorState = ElevatorState.BASE;
                
                trayState = TrayState.BASE;
                isLocked = false;
                climbState = ClimbState.STOP;
                break;

        /************/     

            case CLIMB:
                handlerState = HandlerState.STOP;
                elevatorState = ElevatorState.BASE;
                
                climbState = psController_HID.getOptionsButton() ? ClimbState.CLIMB :
                    psController_HID.getL3Button() ? ClimbState.DESCEND : 
                    psController_HID.getSquareButton() ? ClimbState.STOP :
                        climbState;

                trayState = TrayState.UP;
                break;

        /************/

            case DEPLETE:
                if (elevatorState == ElevatorState.LEVEL3 || Handler.isAlgaeIn())
                    handlerState = HandlerState.DEPLETE_ALGAE;
                else if (elevatorState == ElevatorState.LEVEL0)
                    handlerState =  HandlerState.DEPLETE_CORAL_LEVEL0;
                else
                    handlerState = HandlerState.DEPLETE_CORAL;
                  
                if(Handler.isFinishedDepletingAlgae()) {
                    state = RobotState.INTAKE;
                    elevatorState = ElevatorState.INTAKE_CORAL;
                } else
                    state = RobotState.DEPLETE;

                climbState = ClimbState.STOP;
                trayState = TrayState.BASE;
                break;

        /************/
        
            case INTAKE: 
                if ((elevatorState != ElevatorState.ALGAE_HIGH && elevatorState != ElevatorState.ALGAE_LOW) && !Handler.isCoralIn()) {
                    handlerState = HandlerState.INTAKE_CORAL;
                    elevatorState = ElevatorState.BASE;
                }
                else if (elevatorState == ElevatorState.ALGAE_HIGH || elevatorState == ElevatorState.ALGAE_LOW)
                    handlerState = HandlerState.INTAKE_ALGAE;
                
                state = Handler.isAlgaeIn() || Handler.isCoralIn() ? RobotState.TRAVEL : RobotState.INTAKE;

                climbState = ClimbState.STOP;
                trayState = TrayState.BASE;
                break;
        }

        if ((psController_HID.getR1Button() || psController_HID.getL1Button() || psController_HID.getR3Button()) &&
             Math.abs(Limelight.getTx()) <= 2) // checking for LEDs
            isTxSeen = true;
        else isTxSeen = false;
        
        /****** Auto Counter ******/
        waitForElevatorInAuto(onAuto);


        // ---------- debuging in game
        if (psController_HID.getOptionsButton() && state != RobotState.CLIMB) {
            isMooveCoral = true;
        } else {
            isMooveCoral = false;
        }

        DeliveryManager.operate(elevatorState, state);
        Handler.operate(handlerState);
        Climb.operate(climbState);
        Tray.operate(trayState);
        
        if (isLocked) drivebase.lock();

        lastState = state;
        lastElevatorState = elevatorState;
    }
    
    
    private static void operateAuto(RobotState chosenState, ElevatorState choosenElevatorState) {
        state = chosenState == null ? state : chosenState;
        elevatorState = choosenElevatorState == null ? elevatorState : choosenElevatorState;
        operate(true);
    }
    
    private static void waitForElevatorInAuto(boolean onAuto){
        isAutoElevatorCounting = onAuto && elevatorState != ElevatorState.LEVEL3 && lastElevatorState == ElevatorState.LEVEL3 ? true : isAutoElevatorCounting;

        if (isAutoElevatorCounting && onAuto) autoElevatorCounter++;

        if (autoElevatorCounter < 40 && onAuto && lastElevatorState == ElevatorState.LEVEL3)
            elevatorState = ElevatorState.LEVEL3;
        else {
            isAutoElevatorCounting = false;
            autoElevatorCounter = 0;
        }
    }

    public static SwerveSubsystem getDriveBase() {
        return drivebase;
    }

    public static void setDefaultCommand(Command defaultCommand) {
        drivebase.setDefaultCommand(defaultCommand);
    }

    public static CommandPS4Controller getpsJoystick() {
        return ps4Joystick;
    } 

    public static void initDriveBase() {
        drivebase.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180))); 
    }

    public static RobotState getRobotState() {
        return state;
    }

    public static ElevatorState getElevatorState() {
        return elevatorState;
    }

    public static ElevatorState getLastElevatorState() {
        return lastElevatorState;
    }

    public static HandlerState getHandlerState() {
        return handlerState;
    }

    public static ClimbState getClimbState() {
        return climbState;
    }

    public static boolean getTxSeen() {
        return isTxSeen;
    }

    public static boolean getResetWrist() {
        return isResetWrist;
    }

    public static boolean getIsMooveCoral() {
        return isMooveCoral;
    }
}