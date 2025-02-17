package frc.robot.subsystems.Tray;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Tray {
    private static TalonFX trayMotor = new TalonFX(TrayConstants.TrayMotorID);
    private static double trayPosition = 0;

    private static final MotionMagicVoltage motorRequest = new MotionMagicVoltage(0);

    public static void init() {
        trayMotor.setPosition(0);

        setMotorConfigs();
    }

    public static void operate(TrayState state) {
        switch(state) {
            case BASE:
                trayPosition = TrayConstants.TRAY_POS_BASE;
                break;
            case UP:
                trayPosition = TrayConstants.TRAY_POS_UP;
                break;
        }
        trayMotor.setControl(motorRequest.withPosition(trayPosition));
    }

    private static void setMotorConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = TrayConstants.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = TrayConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = TrayConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = TrayConstants.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = TrayConstants.kI; // no output for integrated error
        slot0Configs.kD = TrayConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = TrayConstants.MotionMagicCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = TrayConstants.MotionMagicAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = TrayConstants.MotionMagicJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = TrayConstants.StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = TrayConstants.SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        trayMotor.getConfigurator().apply(talonFXConfigs);
    }
}
