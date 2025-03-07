package frc.robot.subsystems;

import frc.robot.Dashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Handler.Handler;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class DeliveryManager {
    private static ElevatorState elevatorState = ElevatorState.BASE;
    private static WristState wristState = WristState.BASE;

    public static void init() {
        Wrist.init();
        Elevator.init();
    }

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

            case ALGAE_HIGH:
                if (Elevator.getCurrentPosition() >= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE) {
                    if (Handler.isAlgaeIn()) {
                     elevatorState = ElevatorState.ALGAE_HIGH_IN;
                    }else{
                        wristState = WristState.INTAKE_ALGAE;
                    }
                }
                break;
            
            /*===========================*/
            
            case ALGAE_LOW:
                if (Elevator.getCurrentPosition() >= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE) {
                    if (Handler.isAlgaeIn())
                        elevatorState = ElevatorState.ALGAE_LOW_IN;
                    wristState = WristState.INTAKE_ALGAE;
                }
                break;
                
            /*===========================*/

            case LEVEL0:
                if (Elevator.getCurrentPosition() >= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE)
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
                if (Elevator.getCurrentPosition() >= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE) {
                    wristState = WristState.HIGH;
                }
                break;

            /*===========================*/

            case INTAKE_CORAL:
                wristState = WristState.BASE;
                break;

            case SHOOT_ALGAE:
                if (Elevator.getCurrentPosition() > 10){
                wristState = WristState.SHOOT_ALGAE;
                }
                // else if (Elevator.getCurrentPosition() > 7) {
                //     wristState = WristState.INTAKE_ALGAE;

                // }
        }
            
        wristState = Handler.isAlgaeIn() ? WristState.INTAKE_ALGAE : wristState;
        
        if (Dashboard.getAcceptChages()) {
            elevatorState = Dashboard.getSelectedElevatorState();
            wristState = Dashboard.getSelectedWristState();
        }


        Wrist.operate(wristState);
        Elevator.operate(elevatorState);
    }
        
    public static WristState getWristState() {
        return wristState;
    }

    public static void resetWrist(){
        Wrist.resetEncoder();
    }
}
