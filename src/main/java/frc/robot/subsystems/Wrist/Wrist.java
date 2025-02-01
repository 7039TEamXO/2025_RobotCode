package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Wrist {

    private static TalonFX master = new TalonFX(0);
    private static double wristPosition;

    private static final MotionMagicVoltage motorRequest = new MotionMagicVoltage(0);
    
    public static void init() {
        master.setPosition(0);

        setMotorConfigs();
    }

    public static void operate(WristState state) {
        switch (state) {
            case BASE:
                wristPosition = 0;
                break;

            case HIGH:
                wristPosition = 0;
                break;

            case INTAKE_ALGAE:
                wristPosition = 0;
                break;
                
            case DEPLETE_CORAL:
                wristPosition = 0;
                break;
        }

        master.setControl(motorRequest.withPosition(wristPosition));
    }

    private static void setMotorConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = WristConstants.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = WristConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = WristConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = WristConstants.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = WristConstants.kI; // no output for integrated error
        slot0Configs.kD = WristConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = WristConstants.MotionMagicCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = WristConstants.MotionMagicAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = WristConstants.MotionMagicJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = WristConstants.StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = WristConstants.SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        master.getConfigurator().apply(talonFXConfigs);
    }
}