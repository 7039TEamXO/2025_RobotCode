package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class WristTuning {
    // For the sake of consistency, even though nobody cares about wrist tuning
    private static final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Wrist/kS", WristConstants.kS);
    private static final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Wrist/kV", WristConstants.kV);
    private static final LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Wrist/kA", WristConstants.kA);
    private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Wrist/kP", WristConstants.kP);
    private static final LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Wrist/kI", WristConstants.kI);
    private static final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Wrist/kD", WristConstants.kD);

    private static final LoggedNetworkNumber MotionMagicCruiseVelocity = new LoggedNetworkNumber("/Tuning/Wrist/(Motion Magic) Cruise Velocity", WristConstants.MotionMagicCruiseVelocity);
    private static final LoggedNetworkNumber MotionMagicAcceleration = new LoggedNetworkNumber("/Tuning/Wrist/(Motion Magic) Acceleration", WristConstants.MotionMagicAcceleration);
    private static final LoggedNetworkNumber MotionMagicJerk = new LoggedNetworkNumber("/Tuning/Wrist/(Motion Magic) Jerk", WristConstants.MotionMagicJerk);

    public static final double kS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kS.get() : WristConstants.kS;
    }

    public static final double kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kV.get() : WristConstants.kV;
    }

    public static final double kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kA.get() : WristConstants.kA;
    }

    public static final double kP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kP.get() : WristConstants.kP;
    }

    public static final double kI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kI.get() : WristConstants.kI;
    }

    public static final double kD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kD.get() : WristConstants.kD;
    }

    public static final double MotionMagicCruiseVelocity_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MotionMagicCruiseVelocity.get() : WristConstants.MotionMagicCruiseVelocity;
    }

    public static final double MotionMagicAcceleration_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MotionMagicAcceleration.get() : WristConstants.MotionMagicAcceleration;
    }

    public static final double MotionMagicJerk_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MotionMagicJerk.get() : WristConstants.MotionMagicJerk;
    }
}
