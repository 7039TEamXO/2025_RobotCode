package frc.robot.subsystems;

import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristState;

public class DeliveryManger {

    private static ElevatorState elevatorState = ElevatorState.BASE;
    private static WristState wristStateState = WristState.BASE;

    public static void init(){

    }

    public static void operate(ElevatorState state){
        elevatorState = state;
        switch (state) {
            case BASE:
                wristStateState = WristState.BASE;
                break;

            case ALGAE_HIGH:
                wristStateState = WristState.INTAKE_ALGAE;
                break;

            case ALGAE_LOW:
                wristStateState = WristState.INTAKE_ALGAE;
                break;

            case LEVEL0:
                wristStateState = WristState.BASE;
                break;

            case LEVEL1:
                wristStateState = WristState.BASE;
                break;

            case LEVEL2:
                wristStateState = WristState.BASE;
                break;

            case LEVEL3:
                wristStateState = WristState.HIGH;
                break;
        

        }
        

    }
}
