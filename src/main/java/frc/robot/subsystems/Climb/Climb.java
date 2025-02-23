package frc.robot.subsystems.Climb;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;


public class Climb {
    private static TalonFX climbMotor = new TalonFX(ClimbConstants.ClimbMotorID);
    private static Servo climbServo = new Servo(ClimbConstants.ServoMotorID);
    private static double wantedPower = ClimbConstants.CLIMB_WANTED_POWER_STOP;
    private static double wantedAngle = ClimbConstants.CLIMB_SERVO_CLOSE;

    public static void init() {}

    public static void operate(ClimbState state) {
        switch (state) {
            case STOP:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_STOP;
                wantedAngle = ClimbConstants.CLIMB_SERVO_CLOSE;
                break;
            
            case OPEN:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_STOP;
                wantedAngle = ClimbConstants.CLIMB_SERVO_OPEN;
                break;
            
            case CLIMB:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_CLIMB;
                wantedAngle = ClimbConstants.CLIMB_SERVO_OPEN;
                break;
            
            case DESCEND:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_DESCEND;
                wantedAngle = ClimbConstants.CLIMB_SERVO_OPEN;
                break;
        }

        climbMotor.setControl(new DutyCycleOut(wantedPower));
        climbServo.setAngle(wantedAngle);
    } 

    public static boolean isOpen() {
        return Math.abs(climbServo.getAngle() - ClimbConstants.CLIMB_SERVO_OPEN) < 1;
    }
}
