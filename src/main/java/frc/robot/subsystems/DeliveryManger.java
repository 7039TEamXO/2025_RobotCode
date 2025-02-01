package frc.robot.subsystems;

import frc.robot.Gamepiece;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristState;

public class DeliveryManger {

    private static ElevatorState elevatorState = ElevatorState.BASE;
    private static WristState wristState = WristState.BASE;

    public static void init(){

    }

    public static void operate(ElevatorState state){
        elevatorState = state;
        switch (state) {
            case BASE:
                wristState = WristState.BASE;
                break;

            case ALGAE_HIGH:
                wristState = WristState.INTAKE_ALGAE;
                break;

            case ALGAE_LOW:
                wristState = WristState.INTAKE_ALGAE;
                break;

            case LEVEL0:
                wristState = WristState.BASE;
                break;

            case LEVEL1:
                wristState = WristState.BASE;
                break;

            case LEVEL2:
                wristState = WristState.BASE;
                break;

            case LEVEL3:
                wristState = WristState.HIGH;
                break;
        }

        wristState = SubsystemManager.getGamepiece() == Gamepiece.ALGAE ? WristState.INTAKE_ALGAE : wristState;
        
        

    }
}
