package frc.robot.subsystems;

import java.io.File;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Dashboard;
import frc.robot.Limelight;
import frc.robot.RobotState;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Handler.HandlerState;
import frc.robot.subsystems.IO.ClimbIO;
import frc.robot.subsystems.IO.ElevatorIO;
import frc.robot.subsystems.IO.HandlerIO;
import frc.robot.subsystems.IO.TrayIO;
import frc.robot.subsystems.IO.WristIO;
import frc.robot.subsystems.SwerveDrive.ReefOrientation;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
import frc.robot.subsystems.Tray.Tray;
import frc.robot.subsystems.Tray.TrayState;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Handler.Handler;

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

    private static ElevatorState chosenAlgaeElevatorState = ElevatorState.BASE;
    private static ElevatorState currentAlgaeElevatorState = ElevatorState.BASE;

    // private static int autoElevatorCounter = 0;
    // private static boolean isAutoElevatorCounting = false;
    
    private static HandlerState handlerState = HandlerState.STOP;

    private static ClimbState climbState = ClimbState.STOP;
    private static TrayState trayState = TrayState.BASE;

    private static boolean isTxSeen = false;

    private static boolean isResetWrist = false;

    private static boolean isMoveCoral = false;

    private static boolean isPushClimb = false;

    private static int[] highAlgaeTags = {7, 9, 11, 18, 20, 22};

    public static Command travelCommand = Commands.run(() -> operateAuto(RobotState.TRAVEL, null));
    public static Command intakeCoralCommand = Commands.run(() -> operateAuto(RobotState.INTAKE, ElevatorState.BASE));
    public static Command intakeAlgaeLowCommand = Commands.run(() -> operateAuto(RobotState.INTAKE, ElevatorState.ALGAE_LOW_PROCESSOR));
    public static Command intakeAlgaeHighCommand = Commands.run(() -> operateAuto(RobotState.INTAKE, ElevatorState.ALGAE_HIGH_PROCESSOR));
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

    public static void init(ElevatorIO elevatorIO, HandlerIO handlerIO, WristIO wristIO, ClimbIO climbIO, TrayIO trayIO) {
        state = RobotState.TRAVEL;
        lastState = state;
        DeliveryManager.init(elevatorIO, wristIO);
        Handler.init(handlerIO);
        Climb.init(climbIO);
        Tray.init(trayIO);
    }

    private static Command chooseFeeder;
    private static Command driveToReef;

    public static void operate(boolean onAuto) { 
        if(Constants.CurrentMode != Mode.REAL) simulationPeriodic();

        if (ps4Joystick.getHID().getR3Button() || ps4Joystick.getHID().getR1Button() || ps4Joystick.getHID().getL1Button()) {
            driveToReef = getDrivebase().driveToClosestReefPoint(ps4Joystick.getHID().getR1Button() ? ReefOrientation.RIGHT :
            ps4Joystick.getHID().getL1Button() ? ReefOrientation.LEFT : ReefOrientation.MIDDLE_FAR);
        } else if (driveToReef != null) {
            driveToReef.cancel();
        }

        if (ps4Joystick.R2().getAsBoolean()) {
            chooseFeeder = Handler.isAlgaeInNet() ?
            chooseFeeder = getDrivebase().driveToNet(() -> SubsystemManager.getpsJoystick().getLeftX()) :
                drivebase.chooseFeeder(drivebase.getPose().getY());
            chooseFeeder.schedule();
        } else {
            if (chooseFeeder != null) {
                chooseFeeder.cancel();
            }
        }
        
        if (!onAuto) {
            state = psController_HID.getPOV(0) == 0 ? RobotState.TRAVEL : 
                state == RobotState.CLIMB ? RobotState.CLIMB :
                psController_HID.getL2Button() ? RobotState.DEPLETE :
                psController_HID.getShareButton() ? RobotState.CLIMB :
                    lastState;

            // Ensures that we don't switch to intake when algae's within
            if(!Handler.isAlgaeInProcessor() && !Handler.isAlgaeInNet()) {
                state = psController_HID.getPOV(0) == 90 ? RobotState.INTAKE :
                    psController_HID.getPOV(0) == 270 ? RobotState.INTAKE :
                    psController_HID.getCrossButton() ?
                        RobotState.INTAKE : state;
            }

            elevatorState = state == RobotState.CLIMB ? ElevatorState.BASE :
                psController_HID.getCrossButton() ? ElevatorState.BASE :
                psController_HID.getSquareButton() ? ElevatorState.LEVEL1 :
                psController_HID.getTriangleButton() ? ElevatorState.LEVEL3 :
                psController_HID.getCircleButton() ? ElevatorState.LEVEL2 :
                psController_HID.getPOV(0) == 90 ? ElevatorState.ALGAE_LOW_NET :        // right
                psController_HID.getPOV(0) == 180 ? ElevatorState.LEVEL0 :              // down
                psController_HID.getPOV(0) == 270 ? ElevatorState.ALGAE_LOW_PROCESSOR : // left
                lastElevatorState;
            
            if(Handler.isAlgaeInProcessor() || Handler.isAlgaeInNet()) elevatorState = lastElevatorState;
            
            if(SwerveSubsystem.getClosestReefTag() == highAlgaeTags[0] ||
                SwerveSubsystem.getClosestReefTag() == highAlgaeTags[1] ||
                SwerveSubsystem.getClosestReefTag() == highAlgaeTags[2] ||
                SwerveSubsystem.getClosestReefTag() == highAlgaeTags[3] ||
                SwerveSubsystem.getClosestReefTag() == highAlgaeTags[4] ||
                SwerveSubsystem.getClosestReefTag() == highAlgaeTags[5]) {
                currentAlgaeElevatorState = ElevatorState.ALGAE_HIGH_NET;
            } else {
                currentAlgaeElevatorState = ElevatorState.ALGAE_LOW_NET;
            }

            if ((elevatorState == ElevatorState.ALGAE_LOW_NET || elevatorState == ElevatorState.ALGAE_HIGH_NET ||
                 elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR || elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR) && !Handler.isAlgaeInNet() && !Handler.isAlgaeInProcessor()) {
                chosenAlgaeElevatorState = currentAlgaeElevatorState;
                if(elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR || elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR) {
                    if(currentAlgaeElevatorState == ElevatorState.ALGAE_HIGH_NET) {
                        chosenAlgaeElevatorState = ElevatorState.ALGAE_HIGH_PROCESSOR;
                    } else {
                        chosenAlgaeElevatorState = ElevatorState.ALGAE_LOW_PROCESSOR;
                    }
                }
                if(drivebase.isFarEnoughFromReef()) {
                    elevatorState = chosenAlgaeElevatorState;
                } else {
                    elevatorState = lastElevatorState;
                }
            } else {
                chosenAlgaeElevatorState = ElevatorState.BASE;
            }
        }
        Handler.updateHandlerIR(state, elevatorState, handlerState);
        isPushClimb = false;

        isResetWrist = state != RobotState.CLIMB && psController_HID.getTouchpadButton();

        switch (state) {
            case TRAVEL:
                if(Handler.isAlgaeInProcessor() && (elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR || 
                                                    elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR ||
                                                    elevatorState == ElevatorState.ALGAE_HIGH_IN ||
                                                    elevatorState == ElevatorState.ALGAE_LOW_IN ||
                                                    elevatorState == ElevatorState.BASE)) {
                    if (drivebase.isVeryFarEnoughFromReef()) {
                        elevatorState = ElevatorState.BASE;
                    }
                    handlerState = HandlerState.HOLD_ALGAE;
                }
                else if(Handler.isAlgaeInNet()  && (elevatorState == ElevatorState.ALGAE_LOW_NET || 
                                                    elevatorState == ElevatorState.ALGAE_HIGH_NET ||
                                                    elevatorState == ElevatorState.ALGAE_HOLD_NET ||
                                                    elevatorState == ElevatorState.BASE)) {
                    if (drivebase.isVeryFarEnoughFromReef()) {
                        elevatorState = ElevatorState.BASE;
                    }
                    handlerState = HandlerState.HOLD_NET;
                }
                else {
                    handlerState = HandlerState.STOP;
                }

                if ((lastElevatorState == ElevatorState.INTAKE_CORAL && Handler.isCoralIn()) && elevatorState != ElevatorState.LEVEL0) {
                    elevatorState = ElevatorState.BASE;
                }

                trayState = TrayState.BASE;
                climbState = ClimbState.TRAVEL;
                isLocked = false;
                break;

        /************/     

            case CLIMB:
                handlerState = HandlerState.STOP;
                elevatorState = ElevatorState.BASE;
                
                climbState = psController_HID.getOptionsButton() ? ClimbState.CLIMB :
                    psController_HID.getTouchpadButton() ? ClimbState.DESCEND : 
                    psController_HID.getSquareButton() ? ClimbState.STOP :
                        climbState;
                
                trayState = TrayState.UP;
                
                if (psController_HID.getOptionsButton()) {
                    isPushClimb = true;
                }
                break;

        /************/

            case DEPLETE:
                if (Handler.isAlgaeInNet()) {
                    elevatorState = ElevatorState.ALGAE_HOLD_NET;
                    if (Elevator.getCurrentPosition() > ElevatorConstants.ELEVATOR_POSE_ALGAE_THROW_POS) {
                        handlerState = HandlerState.DEPLETE_NET;
                    } else {
                        handlerState = HandlerState.HOLD_NET;
                    }
                }
                else if (elevatorState == ElevatorState.LEVEL3 || Handler.isAlgaeInProcessor() || 
                    elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR || elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR) {
                    handlerState = HandlerState.DEPLETE_PROCESSOR;
                    if (elevatorState == ElevatorState.LEVEL3 && drivebase.isFarEnoughFromReef()) {
                        state = RobotState.INTAKE;
                        elevatorState = ElevatorState.BASE;
                    }
                }
                else if (elevatorState == ElevatorState.LEVEL0) {
                    handlerState =  HandlerState.DEPLETE_CORAL_LEVEL0;
                    if(drivebase.isFarEnoughFromReef()) {
                        state = RobotState.INTAKE;
                        elevatorState = ElevatorState.BASE;
                    }
                }
                else {
                    handlerState = HandlerState.DEPLETE_CORAL;
                    if(drivebase.isFarEnoughFromReef()) {
                        state = RobotState.INTAKE;
                        elevatorState = ElevatorState.BASE;
                    }
                }
                  
                if(Handler.isFinishedDepletingAlgae()) {
                    state = RobotState.INTAKE;
                    elevatorState = ElevatorState.INTAKE_CORAL;
                } else if(state != RobotState.INTAKE) {
                    state = RobotState.DEPLETE;
                }

                climbState = ClimbState.TRAVEL;
                trayState = TrayState.BASE;
                break;

        /************/
        
            case INTAKE: 
                if ((elevatorState != ElevatorState.ALGAE_HIGH_PROCESSOR && elevatorState != ElevatorState.ALGAE_LOW_PROCESSOR && 
                     elevatorState != ElevatorState.ALGAE_HIGH_NET && elevatorState != ElevatorState.ALGAE_LOW_NET &&
                     chosenAlgaeElevatorState != ElevatorState.ALGAE_HIGH_NET && chosenAlgaeElevatorState != ElevatorState.ALGAE_LOW_NET &&
                     chosenAlgaeElevatorState != ElevatorState.ALGAE_HIGH_PROCESSOR && chosenAlgaeElevatorState != ElevatorState.ALGAE_LOW_PROCESSOR) && !Handler.isCoralIn()) {
                    // *****
                    handlerState = HandlerState.INTAKE_CORAL;
                    elevatorState = ElevatorState.BASE;
                }
                else if(elevatorState == ElevatorState.ALGAE_HIGH_NET || elevatorState == ElevatorState.ALGAE_LOW_NET ||
                     chosenAlgaeElevatorState == ElevatorState.ALGAE_HIGH_NET || chosenAlgaeElevatorState == ElevatorState.ALGAE_LOW_NET ||
                     elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR || elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR ||
                     chosenAlgaeElevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR || chosenAlgaeElevatorState == ElevatorState.ALGAE_LOW_PROCESSOR) {
                        // *****
                    if (elevatorState == chosenAlgaeElevatorState) {
                        // *****
                        if (elevatorState == ElevatorState.ALGAE_HIGH_NET || elevatorState == ElevatorState.ALGAE_LOW_NET) 
                            handlerState = HandlerState.INTAKE_NET;
                        else handlerState = HandlerState.INTAKE_ALGAE;
                        // *****
                        elevatorState = chosenAlgaeElevatorState;
                        driveToReef = getDrivebase().driveToClosestReefPoint(ReefOrientation.MIDDLE);
                    } else {
                        driveToReef = getDrivebase().driveToClosestReefPoint(ReefOrientation.MIDDLE_VERY_FAR);
                    }
                }
                
                state = Handler.isAlgaeInProcessor() || Handler.isCoralIn() || Handler.isAlgaeInNet() ? RobotState.TRAVEL : RobotState.INTAKE;
                climbState = ClimbState.TRAVEL;
                trayState = TrayState.BASE;
                break;
        }

        if ((psController_HID.getR1Button() || psController_HID.getL1Button() || psController_HID.getR3Button()) &&
             Math.abs(Limelight.getTx()) <= 2) // checking for LEDs
            isTxSeen = true;
        else isTxSeen = false;
        
        /****** Auto Counter ******/
        // waitForElevatorInAuto(onAuto);

        if (psController_HID.getOptionsButton() && state != RobotState.CLIMB) {
            isMoveCoral = true;
        } else {
            isMoveCoral = false;
        }

        if (Dashboard.getAcceptChanges()) {
            handlerState = Dashboard.getSelectedHandlerState();
        }

        DeliveryManager.operate(elevatorState, state);
        Handler.operate(handlerState);
        Climb.operate(climbState);
        Tray.operate(trayState);

        if (driveToReef != null && (!ps4Joystick.getHID().getR3Button() && !ps4Joystick.getHID().getR1Button() && 
             !ps4Joystick.getHID().getL1Button() && !(psController_HID.getPOV(0) == 90) && !(psController_HID.getPOV(0) == 270))) {
            driveToReef.cancel();
        } else if(driveToReef != null) {
            driveToReef.schedule();
        }

        if (isLocked) drivebase.lock();

        lastState = state;
        lastElevatorState = elevatorState;
    }
    
    
    private static void operateAuto(RobotState chosenState, ElevatorState choosenElevatorState) {
        state = chosenState == null ? state : chosenState;
        elevatorState = choosenElevatorState == null ? elevatorState : choosenElevatorState;
        operate(true);
    }

    private static void simulationPeriodic() {
        Climb.simulationPeriodic();
        Elevator.simulationPeriodic();
        Handler.simulationPeriodic();
        Tray.simulationPeriodic();
        Wrist.simulationPeriodic();
    }
    
    // private static void waitForElevatorInAuto(boolean onAuto) {
    //     isAutoElevatorCounting = onAuto && elevatorState != ElevatorState.LEVEL3 && lastElevatorState == ElevatorState.LEVEL3 ? true : isAutoElevatorCounting;

    //     if (isAutoElevatorCounting && onAuto) autoElevatorCounter++;

    //     if (autoElevatorCounter < 70 && onAuto && lastElevatorState == ElevatorState.LEVEL3)
    //         elevatorState = ElevatorState.LEVEL3;
    //     else {
    //         isAutoElevatorCounting = false;
    //         autoElevatorCounter = 0;
    //     }
    // }
    
    public static SwerveSubsystem getDrivebase() {
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

    public static ElevatorState getChosenAlgaeElevatorState() {
        return chosenAlgaeElevatorState;
    }

    public static ElevatorState getCurrentAlgaeElevatorState() {
        return currentAlgaeElevatorState;
    }

    public static HandlerState getHandlerState() {
        return handlerState;
    }

    public static ClimbState getClimbState() {
        return climbState;
    }

    public static TrayState getTrayState() {
        return trayState;
    }

    public static boolean getTxSeen() {
        return isTxSeen;
    }

    public static boolean getResetWrist() {
        return isResetWrist;
    }

    public static boolean getIsMoveCoral() {
        return isMoveCoral;
    }

    public static void setState(RobotState sState){
        state = sState;
    }

    public static void setElevatorState(ElevatorState sState){
        elevatorState = sState;
    }

    public static boolean getIsPushClimb() {
        return isPushClimb;
    }
}