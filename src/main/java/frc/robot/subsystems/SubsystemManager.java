package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Dashboard;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SysIdMode;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Handler.HandlerState;
import frc.robot.subsystems.SwerveDrive.ReefOrientation;
import frc.robot.subsystems.SwerveDrive.SwerveDriveTuning;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
import frc.robot.subsystems.Tray.TrayState;

public class SubsystemManager {
    private static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));

    private static final CommandPS4Controller ps4Joystick = new CommandPS4Controller(0);
    private static final PS4Controller psControllerHID = ps4Joystick.getHID();

    // private static boolean isLocked = false;

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

    // Should the Green LED be turned on?
    private static boolean isGreen = false;

    private static boolean isResetWrist = false;

    private static boolean isMoveCoral = false;

    private static boolean isPushClimb = false;

    private static final int[] highAlgaeTags = {7, 9, 11, 18, 20, 22};

    public static void init() {
        state = RobotState.TRAVEL;
        lastState = state;
    }

    private static Command chooseFeeder;
    private static Command driveToReef;
    private static Command testDrive;

    private static boolean isDriveToPoseActive;

    private static void scheduleDriveCommands() {
        if (psControllerHID.getR3Button() || psControllerHID.getR1Button() || psControllerHID.getL1Button()) {
            driveToReef = drivebase.driveToClosestReefPoint(psControllerHID.getR1Button() ? ReefOrientation.RIGHT :
            psControllerHID.getL1Button() ? ReefOrientation.LEFT : ReefOrientation.MIDDLE_FAR);
            isDriveToPoseActive = true;
        } else if (driveToReef != null) {
            driveToReef.cancel();
        }

        if (psControllerHID.getR2Button()) {
            chooseFeeder = RobotContainer.handler.isAlgaeInNet() ?
                drivebase.driveToNet() :
                RobotContainer.handler.isAlgaeInProcessor() ?
                drivebase.driveToProcessor() :
                drivebase.chooseFeeder(drivebase.getPose().getY());
            chooseFeeder.schedule();
            isDriveToPoseActive = true;
        } else {
            if (chooseFeeder != null) {
                chooseFeeder.cancel();
            }
        }

        // solely to be used for DriveToPose testing
        if(psControllerHID.getL3Button()) {
            testDrive = drivebase.driveToPoseAlt(new Pose2d(SwerveDriveTuning.TEST_DRIVE_X_get(), SwerveDriveTuning.TEST_DRIVE_Y_get(),
                new Rotation2d(Math.toRadians(SwerveDriveTuning.TEST_DRIVE_ANGLE_get()))));
            testDrive.schedule();
        } else {
            if(testDrive != null) {
                testDrive.cancel();
            }
        }
    }

    private static void operateDriveToReef() {
        if (driveToReef != null && (!psControllerHID.getR3Button() && !psControllerHID.getR1Button() && 
             !psControllerHID.getL1Button() && !(psControllerHID.getPOV(0) == 90) && !(psControllerHID.getPOV(0) == 270))) {
            driveToReef.cancel();
        } else if(driveToReef != null) {
            isDriveToPoseActive = true;
            driveToReef.schedule();
        }
    }

    private static void getButtonStates() {
        state = psControllerHID.getPOV(0) == 0 ? RobotState.TRAVEL : 
            state == RobotState.CLIMB ? RobotState.CLIMB :
            psControllerHID.getL2Button() ? RobotState.DEPLETE :
            psControllerHID.getShareButton() ? RobotState.CLIMB :
            lastState;

        // Ensures that we don't switch to intake when algae's within
        if(!RobotContainer.handler.isAlgaeInProcessor() && !RobotContainer.handler.isAlgaeInNet()) {
            state = psControllerHID.getPOV(0) == 90 ? RobotState.INTAKE :
                psControllerHID.getPOV(0) == 270 ? RobotState.INTAKE :
                psControllerHID.getCrossButton() ?
                    RobotState.INTAKE : state;
        }

        elevatorState = state == RobotState.CLIMB ? ElevatorState.BASE :
            psControllerHID.getCrossButton() ? ElevatorState.BASE :
            psControllerHID.getSquareButton() ? ElevatorState.LEVEL1 :
            psControllerHID.getTriangleButton() ? ElevatorState.LEVEL3 :
            psControllerHID.getCircleButton() ? ElevatorState.LEVEL2 :
            psControllerHID.getPOV(0) == 90 ? ElevatorState.ALGAE_LOW_NET :        // right
            psControllerHID.getPOV(0) == 180 ? ElevatorState.LEVEL0 :              // down
            psControllerHID.getPOV(0) == 270 ? ElevatorState.ALGAE_LOW_PROCESSOR : // left
            lastElevatorState;

        if(RobotContainer.handler.isAlgaeInProcessor() || RobotContainer.handler.isAlgaeInNet()) elevatorState = lastElevatorState;
    }

    private static void determineAlgaeStates() {
        if(drivebase.getClosestReefTag() == highAlgaeTags[0] ||
            drivebase.getClosestReefTag() == highAlgaeTags[1] ||
            drivebase.getClosestReefTag() == highAlgaeTags[2] ||
            drivebase.getClosestReefTag() == highAlgaeTags[3] ||
            drivebase.getClosestReefTag() == highAlgaeTags[4] ||
            drivebase.getClosestReefTag() == highAlgaeTags[5]) {
            currentAlgaeElevatorState = ElevatorState.ALGAE_HIGH_NET;
        } else {
            currentAlgaeElevatorState = ElevatorState.ALGAE_LOW_NET;
        }

        if ((elevatorState == ElevatorState.ALGAE_LOW_NET || elevatorState == ElevatorState.ALGAE_HIGH_NET ||
            elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR || elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR) && !RobotContainer.handler.isAlgaeInNet() && !RobotContainer.handler.isAlgaeInProcessor()) {
            chosenAlgaeElevatorState = currentAlgaeElevatorState;
            if(elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR || elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR) {
                if(currentAlgaeElevatorState == ElevatorState.ALGAE_HIGH_NET) {
                    chosenAlgaeElevatorState = ElevatorState.ALGAE_HIGH_PROCESSOR;
                } else {
                    chosenAlgaeElevatorState = ElevatorState.ALGAE_LOW_PROCESSOR;
                }
            }
            if(drivebase.isFarFromReef()) {
                elevatorState = chosenAlgaeElevatorState;
            } else {
                elevatorState = lastElevatorState;
            }
        } else {
            chosenAlgaeElevatorState = ElevatorState.BASE;
        }
    }

    /* ************************* */

    public static void operate(boolean onAuto) { 
        if(Constants.CurrentMode != Mode.REAL) simulationPeriodic();

        isDriveToPoseActive = false;
        
        if (!onAuto) {
            if(Constants.CurrentSysIdMode == SysIdMode.INACTIVE) scheduleDriveCommands();

            getButtonStates(); 
            
            determineAlgaeStates();
        }

        RobotContainer.handler.updateHandlerIR(state, elevatorState, handlerState);
        isPushClimb = false;

        isResetWrist = state != RobotState.CLIMB && psControllerHID.getTouchpadButton();

        switch (state) {
            case TRAVEL:
                if(RobotContainer.handler.isAlgaeInProcessor() && (elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR || 
                                                    elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR ||
                                                    elevatorState == ElevatorState.ALGAE_HIGH_IN ||
                                                    elevatorState == ElevatorState.ALGAE_LOW_IN ||
                                                    elevatorState == ElevatorState.BASE)) {
                    if (drivebase.isVeryFarFromReef()) {
                        elevatorState = ElevatorState.BASE;
                    }
                    handlerState = HandlerState.HOLD_ALGAE;
                }
                else if(RobotContainer.handler.isAlgaeInNet()  && (elevatorState == ElevatorState.ALGAE_LOW_NET || 
                                                    elevatorState == ElevatorState.ALGAE_HIGH_NET ||
                                                    elevatorState == ElevatorState.ALGAE_HOLD_NET ||
                                                    elevatorState == ElevatorState.BASE)) {
                    if (drivebase.isVeryFarFromReef()) {
                        elevatorState = ElevatorState.BASE;
                    }
                    handlerState = HandlerState.HOLD_NET;
                }
                else {
                    handlerState = HandlerState.STOP;
                }

                if ((lastElevatorState == ElevatorState.INTAKE_CORAL && RobotContainer.handler.isCoralIn()) && elevatorState != ElevatorState.LEVEL0) {
                    elevatorState = ElevatorState.BASE;
                }

                trayState = TrayState.BASE;
                climbState = ClimbState.TRAVEL;
                // isLocked = false;
                break;

        /************/     

            case CLIMB:
                handlerState = HandlerState.STOP;
                elevatorState = ElevatorState.BASE;
                
                climbState = psControllerHID.getOptionsButton() ? ClimbState.CLIMB :
                    psControllerHID.getTouchpadButton() ? ClimbState.DESCEND : 
                    psControllerHID.getSquareButton() ? ClimbState.STOP :
                        climbState;
                
                trayState = TrayState.UP;
                
                if (psControllerHID.getOptionsButton()) {
                    isPushClimb = true;
                }
                break;

        /************/

            case DEPLETE:
                if (RobotContainer.handler.isAlgaeInNet()) {
                    elevatorState = ElevatorState.ALGAE_HOLD_NET;
                    if (RobotContainer.elevator.getCurrentPosition() > ElevatorConstants.ELEVATOR_POSE_ALGAE_THROW_POS) {
                        handlerState = HandlerState.DEPLETE_NET;
                    } else {
                        handlerState = HandlerState.HOLD_NET;
                    }
                }
                else if (elevatorState == ElevatorState.LEVEL3 || RobotContainer.handler.isAlgaeInProcessor() || 
                    elevatorState == ElevatorState.ALGAE_HIGH_PROCESSOR || elevatorState == ElevatorState.ALGAE_LOW_PROCESSOR) {
                    handlerState = HandlerState.DEPLETE_PROCESSOR;
                    if (elevatorState == ElevatorState.LEVEL3 && drivebase.isFarFromReef()) {
                        state = RobotState.INTAKE;
                        elevatorState = ElevatorState.BASE;
                    }
                }
                else if (elevatorState == ElevatorState.LEVEL0) {
                    handlerState =  HandlerState.DEPLETE_CORAL_LEVEL0;
                    if(drivebase.isFarFromReef()) {
                        state = RobotState.INTAKE;
                        elevatorState = ElevatorState.BASE;
                    }
                }
                else {
                    handlerState = HandlerState.DEPLETE_CORAL;
                    if(drivebase.isFarFromReef()) {
                        state = RobotState.INTAKE;
                        elevatorState = ElevatorState.BASE;
                    }
                }
                  
                if(RobotContainer.handler.isFinishedDepletingAlgae()) {
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
                     chosenAlgaeElevatorState != ElevatorState.ALGAE_HIGH_PROCESSOR && chosenAlgaeElevatorState != ElevatorState.ALGAE_LOW_PROCESSOR) && !RobotContainer.handler.isCoralIn()) {
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
                        driveToReef = drivebase.driveToClosestReefPoint(ReefOrientation.MIDDLE);
                    } else {
                        driveToReef = drivebase.driveToClosestReefPoint(ReefOrientation.MIDDLE_VERY_FAR);
                    }
                }
                
                state = RobotContainer.handler.isAlgaeInProcessor() || RobotContainer.handler.isCoralIn() || RobotContainer.handler.isAlgaeInNet() ? RobotState.TRAVEL : RobotState.INTAKE;
                climbState = ClimbState.TRAVEL;
                trayState = TrayState.BASE;
                break;
        }

        // Green-light LED check
        isGreen = (psControllerHID.getR1Button() || psControllerHID.getL1Button() || psControllerHID.getR3Button()) &&
            Math.abs(Limelight.getTX()) <= 2;
        
        /****** Auto Counter ******/
        // waitForElevatorInAuto(onAuto);

        if (psControllerHID.getOptionsButton() && state != RobotState.CLIMB) {
            isMoveCoral = true;
        } else {
            isMoveCoral = false;
        }

        if (Dashboard.getAcceptChanges()) {
            handlerState = Dashboard.getSelectedHandlerState();
        }

        if(Constants.CurrentSysIdMode == SysIdMode.INACTIVE) {
            DeliveryManager.operate(elevatorState, state);
            RobotContainer.handler.operate(handlerState);
            RobotContainer.climb.operate(climbState);
            RobotContainer.tray.operate(trayState);

            // Operate on the automatic drive to reef
            operateDriveToReef();
        }

        // if (isLocked) drivebase.lock();

        lastState = state;
        lastElevatorState = elevatorState;
    }
    
    
    public static Command operateAuto(RobotState chosenState, ElevatorState chosenElevatorState) {
        return new InstantCommand(() -> {
            state = chosenState == null ? state : chosenState;
            elevatorState = chosenElevatorState == null ? elevatorState : chosenElevatorState;
        });
    }

    private static void simulationPeriodic() {
        RobotContainer.climb.simulationPeriodic();
        RobotContainer.elevator.simulationPeriodic();
        RobotContainer.handler.simulationPeriodic();
        RobotContainer.tray.simulationPeriodic();
        RobotContainer.wrist.simulationPeriodic();
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

    public static CommandPS4Controller getPSJoystick() {
        return ps4Joystick;
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

    public static boolean getIsGreen() {
        return isGreen;
    }

    public static boolean getResetWrist() {
        return isResetWrist;
    }

    public static boolean getIsMoveCoral() {
        return isMoveCoral;
    }

    public static boolean getIsPushClimb() {
        return isPushClimb;
    }

    public static boolean isDriveToPoseActive() {
        return isDriveToPoseActive;
    }

    public static void setStateToDefault() {
        state = RobotState.TRAVEL;
    }
}