package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Handler.HandlerState;

public class Wrist {

    private static TalonFX master = new TalonFX(0);
    private static double wristPosition;

    private static final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    
    public static void init(){
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        master.getConfigurator().apply(talonFXConfigs);

    }

    public static void operate(WristState state){
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


        }

        master.setControl(m_request.withPosition(wristPosition));


    }
}
