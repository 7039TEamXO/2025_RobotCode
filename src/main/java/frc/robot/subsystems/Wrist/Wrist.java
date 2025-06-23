package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Dashboard;
import frc.robot.Constants.Mode;
import frc.robot.Constants.TuningMode;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Handler.Handler;
import frc.robot.subsystems.IO.WristIO;
import frc.robot.subsystems.IO.WristIO.WristIOInputs;

public class Wrist {
    private static WristIO io;
    private static WristIOInputs inputs = new WristIOInputs();

    private static double wristPosition;

    private static int baseCounter = 0;

    private static boolean isMoveWrist = false;

    private static WristState lastState = WristState.BASE;

    private static final MotionMagicVoltage motorRequest = new MotionMagicVoltage(0);
    
    public static void init(WristIO _io) {
        io = _io;

        io.setPosition(0);

        setMotorConfigs();

        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    public static void operate(WristState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        if(Constants.GetTuningMode() == TuningMode.ACTIVE) setMotorConfigs();

        switch (state) {
            case BASE:
                wristPosition = WristConstants.WRIST_POS_BASE;
                break;

            case HIGH:
                wristPosition = WristConstants.WRIST_POS_HIGH;
                break;

            case HOLD_ALGAE_PROCESSOR:
                wristPosition = WristConstants.WRIST_POS_INTAKE_ALGAE;
                break;
                
            case DEPLETE_CORAL:
                wristPosition = WristConstants.WRIST_POS_DEPLETE_CORAL;
                break;

            case DEPLETE_CORAL_LEVEL0:
                wristPosition = WristConstants.WRIST_POS_DEPLETE_CORAL_LEVEL0;
                break;

            case HOLD_ALGAE_NET:
                wristPosition = WristConstants.WRIST_POS_HOLD_ALGAE_NET;
                break;

            case THROW_ALGAE_NET:
                wristPosition = WristConstants.THROW_ALGAE_NET;
                break;
        }

        if(lastState != state) {
            isMoveWrist = true;
        }

        if (state == WristState.BASE) {
            baseCounter++;
        } else {
            baseCounter = 0;
        }
        if (inputs.currentAmps > 27 && baseCounter > 10 && !Handler.isAlgaeInNet() && !Handler.isAlgaeInProcessor() && getCurrentVelocity() < 1) {
            io.setPosition(0);
        }

        if ((Constants.CurrentMode != Mode.SIM) && (state == WristState.BASE && inputs.currentAmps < 28 && isMoveWrist) && (!Handler.isAlgaeInProcessor() && !Handler.isAlgaeInNet())){
            io.setMotionMagic(new DutyCycleOut(-0.2));
        } else {
            isMoveWrist = false;
            io.setMotionMagic(motorRequest.withPosition(wristPosition + Dashboard.addValueToWrist()));
        }

        lastState = state;
        if (SubsystemManager.getResetWrist()) {
            if(Constants.CurrentMode != Mode.SIM) io.setMotionMagic(new DutyCycleOut(-0.2));
            io.setPosition(0);
        }
    }

    public static double getCurrentPosition() {
        return inputs.position;
    }

    public static void resetEncoder() {
        io.setPosition(0);
    }

    public static boolean isWristAtSetPoint() {
        return Math.abs(wristPosition - inputs.position) < WristConstants.WRIST_POS_TOLERANCE;
    }

    public static double getCurrentVelocity() {
        return Math.abs(inputs.velocity);
    }

    public static double getCurrentVoltage() {
        return Math.abs(inputs.appliedVolts);
    }

    private static void setMotorConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = WristTuning.kS_get(); // Add 0.25 V output to overcome static friction
        slot0Configs.kV = WristTuning.kV_get(); // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = WristTuning.kA_get(); // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = WristTuning.kP_get(); // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = WristTuning.kI_get(); // no output for integrated error
        slot0Configs.kD = WristTuning.kD_get(); // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = WristTuning.MotionMagicCruiseVelocity_get(); // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = WristTuning.MotionMagicAcceleration_get(); // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = WristTuning.MotionMagicJerk_get(); // Target jerk of 1600 rps/s/s (0.1 seconds)
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = WristConstants.StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = WristConstants.SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyTalonFXConfig(talonFXConfigs);
    }

    public static void simulationPeriodic() {
        io.simulationPeriodic();
    }
}