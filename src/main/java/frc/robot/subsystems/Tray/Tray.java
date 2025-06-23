package frc.robot.subsystems.Tray;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;
import frc.robot.Robot;
import frc.robot.subsystems.IO.TrayIO;
import frc.robot.subsystems.IO.TrayIO.TrayIOInputs;

public class Tray {
    private static TrayIO io;
    private static TrayIOInputs inputs = new TrayIOInputs();

    // private static TalonFX trayMotor = new TalonFX(TrayConstants.TrayMotorID);
    private static double trayPosition = 0;

    // private static double startCounter = 0;
    // private static boolean isStartGame = true;

    private static final MotionMagicVoltage motorRequest = new MotionMagicVoltage(0);

    public static void init(TrayIO _io) {
        io = _io;

        io.setPosition(0);

        setMotorConfigs();

        io.updateInputs(inputs);
        Logger.processInputs("Tray", inputs);
    }

    public static void operate(TrayState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Tray", inputs);

        if(Constants.CurrentTuningMode == TuningMode.TUNE) setMotorConfigs();

        switch(state) {
            case BASE:
                trayPosition = TrayConstants.TRAY_POS_BASE;
                break;
            case UP:
                trayPosition = TrayConstants.TRAY_POS_UP;
                break;
        }
        
        if(Robot.isAuto()) {
            // if(startCounter <= 150) {
            //     startCounter ++;
            //     trayMotor.setControl(new DutyCycleOut(-0.1));
            //     if(startCounter == 150) {
            //         trayMotor.setPosition(0);
            //         trayMotor.setControl(motorRequest.withPosition(trayPosition));
            //         startCounter ++;
            //     }
            // } else {
            //     trayMotor.setControl(motorRequest.withPosition(trayPosition));
            // }
        } else {
            // System.out.println(trayMotor.getPosition().getValueAsDouble());
            io.setMotionMagic(motorRequest.withPosition(trayPosition));

        // }
        // if(SubsystemManager.getpsJoystick().getHID().getTouchpadButton() && state != TrayState.UP) {
        //     trayMotor.setControl(new DutyCycleOut(-0.1));
        //     trayMotor.setPosition(0);
        }
    }

    private static void setMotorConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = TrayTuning.kS_get(); // Add 0.25 V output to overcome static friction
        slot0Configs.kV = TrayTuning.kV_get(); // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = TrayTuning.kA_get(); // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = TrayTuning.kP_get(); // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = TrayTuning.kI_get(); // no output for integrated error
        slot0Configs.kD = TrayTuning.kD_get(); // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = TrayTuning.MotionMagicCruiseVelocity_get(); // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = TrayTuning.MotionMagicAcceleration_get(); // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = TrayTuning.MotionMagicJerk_get(); // Target jerk of 1600 rps/s/s (0.1 seconds)
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = TrayConstants.StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = TrayConstants.SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyTalonFXConfig(talonFXConfigs);
    }

    public static double getCurrentPosition(){
        return inputs.position;
    }

    public static double getCurrentVelocity() {
        return inputs.velocity;
    }

    public static double getCurrentVoltage() {
        return inputs.appliedVolts;
    }

    public static void simulationPeriodic() {
        io.simulationPeriodic();
    }
}
