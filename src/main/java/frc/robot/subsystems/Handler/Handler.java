package frc.robot.subsystems.Handler;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Elevator.ElevatorState;

public class Handler {

    private static TalonFX master = new TalonFX(0);
    
    public static void init(){

    }

    public static void operate(HandlerState state){
        switch (state) {
            case STOP:
                
                break;
            case DEPLETE:

                break;
            case INTAKE:

                break;
        
        }
    }   
}
