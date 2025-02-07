package frc.robot.subsystems;

import frc.robot.Gamepiece;
import frc.robot.subsystems.Elevator.Elevator;
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
                wristState = WristState.DEPLETE_CORAL_LEVEL0;
                break;

            case LEVEL1:
                wristState = WristState.DEPLETE_CORAL;
                break;

            case LEVEL2:
                wristState = WristState.DEPLETE_CORAL;
                break;

            case LEVEL3:
                wristState = WristState.HIGH;
                break;
        }

        wristState = Handler.isAlgaeIn() ? WristState.INTAKE_ALGAE : wristState;

        Wrist.operate(wristState);
        Elevator.operate(elevatorState);
    }
}
