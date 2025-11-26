package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.SysIdMode;
import frc.robot.Dashboard;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristState;

public class DeliveryManager {
    private static ElevatorState elevatorState = ElevatorState.BASE;
    private static WristState wristState = WristState.BASE;

    public static ElevatorState getElevatorState() {
        return elevatorState;
    }
    
    public static void operate(ElevatorState state, RobotState robotState) {
        elevatorState = state;

        switch (state) {
            
            /*===========================*/
            
            case BASE:
                    wristState = WristState.BASE;
                break;

            /*===========================*/

            case ALGAE_HIGH_IN:
            case ALGAE_HIGH_PROCESSOR:
                if (RobotContainer.elevator.getCurrentPosition() >= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE) {
                    if (RobotContainer.handler.isAlgaeInProcessor()) {
                        elevatorState = ElevatorState.ALGAE_HIGH_IN;
                    } else{
                        wristState = WristState.HOLD_ALGAE_PROCESSOR;
                    }
                }
                break;
            
            /*===========================*/
            
            case ALGAE_LOW_IN:
            case ALGAE_LOW_PROCESSOR:
                if (RobotContainer.elevator.getCurrentPosition() >= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE) {
                    if (RobotContainer.handler.isAlgaeInProcessor())
                        elevatorState = ElevatorState.ALGAE_LOW_IN;
                    wristState = WristState.HOLD_ALGAE_PROCESSOR;
                }
                break;
                
            /*===========================*/

            case LEVEL0:
                if (RobotContainer.elevator.getCurrentPosition() >= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE)
                    wristState = WristState.DEPLETE_CORAL_LEVEL0;
                else
                    elevatorState = wristState != WristState.DEPLETE_CORAL_LEVEL0 ? ElevatorState.LEVEL1 : ElevatorState.LEVEL0;
                break;

            /*===========================*/

            case LEVEL1:
                wristState = WristState.DEPLETE_CORAL;
                break;
            
            /*===========================*/

            case LEVEL2:
                wristState = WristState.DEPLETE_CORAL;
                break;
            
            /*===========================*/

            case LEVEL3:
                if (RobotContainer.elevator.getCurrentPosition() >= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE) {
                    wristState = WristState.HIGH;
                }
                break;

            /*===========================*/

            case INTAKE_CORAL:
                wristState = WristState.BASE;
                break;

            case ALGAE_HIGH_NET:
                wristState = WristState.HOLD_ALGAE_NET;
                break;

            case ALGAE_LOW_NET:
                wristState = WristState.HOLD_ALGAE_NET;
                break;
                
            case ALGAE_HOLD_NET:
                elevatorState = ElevatorState.LEVEL3;
                wristState = WristState.THROW_ALGAE_NET;
                break;
        }
            
        wristState = RobotContainer.handler.isAlgaeInProcessor() ? WristState.HOLD_ALGAE_PROCESSOR : wristState;
        wristState = RobotContainer.handler.isAlgaeInNet() && wristState != WristState.THROW_ALGAE_NET ? WristState.HOLD_ALGAE_NET : wristState;
        
        if (Dashboard.getAcceptChanges()) {
            elevatorState = Dashboard.getSelectedElevatorState();
            wristState = Dashboard.getSelectedWristState();
        }

        if(Constants.CurrentSysIdMode == SysIdMode.INACTIVE) {
            RobotContainer.wrist.operate(wristState);
            RobotContainer.elevator.operate(elevatorState);
        }
    }
        
    public static WristState getWristState() {
        return wristState;
    }

    public static void resetWrist(){
        RobotContainer.wrist.resetEncoder();
    }
}
