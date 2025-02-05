package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator {

    private static double elevatorPosition; 
    private static final MotionMagicVoltage motorRequest = new MotionMagicVoltage(0);

    private static TalonFX elevatorMotor = new TalonFX(ElevatorConstants.ElevatorRightMotorID);
    
    public static void init() {
        elevatorMotor.setPosition(0); // we start our position from 0

        setMotorConfigs();
    }

    public static void operate(ElevatorState state){
        switch (state) {
            case BASE: 
                elevatorPosition = 0;
                break;

            case ALGAE_HIGH:
                elevatorPosition = 0;
                break;

            case ALGAE_LOW:
                elevatorPosition = 0;
                break;

            case LEVEL0:
                elevatorPosition = 0;
                break;

            case LEVEL1:
                elevatorPosition = 0;
                break;

            case LEVEL2:
                elevatorPosition = 0;
                break;

            case LEVEL3:
                elevatorPosition = 0;
                break;

        }

        elevatorMotor.setControl(motorRequest.withPosition(elevatorPosition)); //set position for elevator
    }

    public static double getCurrentPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    public static double getCmFromEncoder(double encoder) {
        return encoder * ElevatorConstants.EncoderMultiplier;
    }

    private static void setMotorConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ElevatorConstants.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = ElevatorConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = ElevatorConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = ElevatorConstants.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = ElevatorConstants.kI; // no output for integrated error
        slot0Configs.kD = ElevatorConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MotionMagicJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        elevatorMotor.getConfigurator().apply(talonFXConfigs);
    }
}
