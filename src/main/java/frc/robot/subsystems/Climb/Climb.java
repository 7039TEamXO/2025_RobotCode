package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.hardware.TalonFX;


public class Climb {
    private static TalonFX climbMotor = new TalonFX(ClimbConstants.ClimbMotorID);
    private static double wantedPower = 0;


    public static void init() {

    }

    public static void operate(ClimbState state) {
        switch (state) {
            case STOP:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_STOP;
                break;
            case UP:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_UP;
                break;
            case DOWN:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_DOWN;
                break;
        }

        climbMotor.setControl(new DutyCycleOut(wantedPower));
    } 


}
