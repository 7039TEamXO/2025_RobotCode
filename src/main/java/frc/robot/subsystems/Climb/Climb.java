package frc.robot.subsystems.Climb;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;


public class Climb {
    private static TalonFX climbMotor = new TalonFX(ClimbConstants.ClimbMotorID);
    private static Servo climbServo = new Servo(ClimbConstants.ServoMotorID);
    private static double wantedPower = ClimbConstants.CLIMB_WANTED_POWER_STOP;
    private static double wantedAngle = ClimbConstants.CLIMB_SERVO_CLOSE;

    public static void init() {
        setMotorConfigs();
        climbMotor.setPosition(0);
    }

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
        if (climbMotor.getPosition().getValueAsDouble() >= 220 && wantedPower == ClimbConstants.CLIMB_WANTED_POWER_CLIMB) {  //climbMotor.getPosition().getValueAsDouble() <= -220 && wantedPower == ClimbConstants.CLIMB_WANTED_POWER_DESCEND
            wantedPower = ClimbConstants.CLIMB_WANTED_POWER_STOP;
        }
        climbMotor.setControl(new DutyCycleOut(wantedPower));
        climbServo.setAngle(wantedAngle);
    } 

    private static void setMotorConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ClimbConstants.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = ClimbConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = ClimbConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = ClimbConstants.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = ClimbConstants.kI; // no output for integrated error
        slot0Configs.kD = ClimbConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.MotionMagicCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.MotionMagicAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = ClimbConstants.MotionMagicJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ClimbConstants.StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        climbMotor.getConfigurator().apply(talonFXConfigs);
    }

    public static boolean isOpen() {
        return Math.abs(climbServo.getAngle() - ClimbConstants.CLIMB_SERVO_OPEN) < 1;
    }
    public static double getClimbPose() {
        return climbMotor.getPosition().getValueAsDouble();
    }
}
