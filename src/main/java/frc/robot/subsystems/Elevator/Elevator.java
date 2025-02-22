package frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Elevator {
    private static double elevatorPosition; 
    private static final MotionMagicVoltage motorRequest = new MotionMagicVoltage(0);

    private static TalonFX elevatorMasterMotor = new TalonFX(ElevatorConstants.ElevatorRightMotorID);
    private static TalonFX elevatorSlaveMotor = new TalonFX(ElevatorConstants.ElevatorLeftMotorID);
    
    public static void init() {
        elevatorMasterMotor.setPosition(0); // we start our position from 0
        elevatorSlaveMotor.setPosition(0); // we start our position from 0
        setMotorConfigs();
    }

    public static void operate(ElevatorState state){
        switch (state) {
            case BASE: 
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_BASE;
                break;

            case ALGAE_HIGH:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_HIGH;
                break;

            case ALGAE_LOW:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_LOW;
                break;

            case LEVEL0:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_LEVEL0;
                break;

            case LEVEL1:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_LEVEL1;
                break;

            case LEVEL2:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_LEVEL2;
                break;

            case LEVEL3:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_LEVEL3;
                break;

            case INTAKE_CORAL:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_INTAKE_CORAL;
                break;

            case ALGAE_HIGH_IN:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_HIGH + ElevatorConstants.ELEVATOR_POSE_ALGAE_IN_OFFSET;
                break;

            case ALGAE_LOW_IN:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_LOW + ElevatorConstants.ELEVATOR_POSE_ALGAE_IN_OFFSET;
                break;

            default:
                break;

        }

        elevatorMasterMotor.setControl(motorRequest.withPosition(elevatorPosition)); //set position for elevator
    }

    public static double getCurrentPosition() {
        return elevatorMasterMotor.getPosition().getValueAsDouble();
    }

    public static double getCmFromEncoder(double encoder) {
        return encoder * ElevatorConstants.EncoderMultiplier;
    }

    public static double getMotorOutput(){
        return elevatorMasterMotor.getVelocity().getValueAsDouble();
    }

    private static void setMotorConfigs() {
        elevatorSlaveMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), true));

        var rightTalonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = rightTalonFXConfigs.Slot0;
        slot0Configs.kS = ElevatorConstants.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = ElevatorConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = ElevatorConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = ElevatorConstants.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = ElevatorConstants.kI; // no output for integrated error
        slot0Configs.kD = ElevatorConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = rightTalonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MotionMagicJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
        
        rightTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightTalonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorCurrentLimit;
        rightTalonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightTalonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SupplyCurrentLimit;
        rightTalonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        rightTalonFXConfigs.MotorOutput.PeakForwardDutyCycle = 1;
        rightTalonFXConfigs.MotorOutput.PeakReverseDutyCycle = -0.6;

        elevatorMasterMotor.getConfigurator().apply(rightTalonFXConfigs);

        var leftTalonFXConfigs = new TalonFXConfiguration();

        leftTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftTalonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftTalonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorCurrentLimit;
        leftTalonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SupplyCurrentLimit;
        leftTalonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftTalonFXConfigs.MotorOutput.PeakForwardDutyCycle = 1;
        leftTalonFXConfigs.MotorOutput.PeakReverseDutyCycle = -0.6;

        elevatorSlaveMotor.getConfigurator().apply(leftTalonFXConfigs);
    }
}
