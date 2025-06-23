package frc.robot.subsystems.Tray;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class TrayTuning {
    // For the sake of consistency, even though nobody cares about tray tuning
    private static final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Tray/kS", TrayConstants.kS);
    private static final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Tray/kV", TrayConstants.kV);
    private static final LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Tray/kA", TrayConstants.kA);
    private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Tray/kP", TrayConstants.kP);
    private static final LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Tray/kI", TrayConstants.kI);
    private static final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Tray/kD", TrayConstants.kD);

    private static final LoggedNetworkNumber MotionMagicCruiseVelocity = new LoggedNetworkNumber("/Tuning/Tray/MotionMagicCruiseVelocity", TrayConstants.MotionMagicCruiseVelocity);
    private static final LoggedNetworkNumber MotionMagicAcceleration = new LoggedNetworkNumber("/Tuning/Tray/MotionMagicAcceleration", TrayConstants.MotionMagicAcceleration);
    private static final LoggedNetworkNumber MotionMagicJerk = new LoggedNetworkNumber("/Tuning/Tray/MotionMagicJerk", TrayConstants.MotionMagicJerk);

    public static final double kS_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kS.get() : TrayConstants.kS;
    }

    public static final double kV_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kV.get() : TrayConstants.kV;
    }

    public static final double kA_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kA.get() : TrayConstants.kA;
    }

    public static final double kP_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kP.get() : TrayConstants.kP;
    }

    public static final double kI_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kI.get() : TrayConstants.kI;
    }

    public static final double kD_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kD.get() : TrayConstants.kD;
    }

    public static final double MotionMagicCruiseVelocity_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? MotionMagicCruiseVelocity.get() : TrayConstants.MotionMagicCruiseVelocity;
    }

    public static final double MotionMagicAcceleration_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? MotionMagicAcceleration.get() : TrayConstants.MotionMagicAcceleration;
    }

    public static final double MotionMagicJerk_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? MotionMagicJerk.get() : TrayConstants.MotionMagicJerk;
    }
}
