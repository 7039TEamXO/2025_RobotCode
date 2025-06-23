package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class ClimbTuning {
    private static final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Climb/kS", ClimbConstants.kS);
    private static final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Climb/kV", ClimbConstants.kV);
    private static final LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Climb/kA", ClimbConstants.kA);
    private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Climb/kP", ClimbConstants.kP);
    private static final LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Climb/kI", ClimbConstants.kI);
    private static final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Climb/kD", ClimbConstants.kD);

    private static final LoggedNetworkNumber MotionMagicCruiseVelocity = new LoggedNetworkNumber("/Tuning/Climb/MotionMagicCruiseVelocity", ClimbConstants.MotionMagicCruiseVelocity);
    private static final LoggedNetworkNumber MotionMagicAcceleration = new LoggedNetworkNumber("/Tuning/Climb/MotionMagicAcceleration", ClimbConstants.MotionMagicAcceleration);
    private static final LoggedNetworkNumber MotionMagicJerk = new LoggedNetworkNumber("/Tuning/Climb/MotionMagicJerk", ClimbConstants.MotionMagicJerk);

    private static final LoggedNetworkNumber CLIMB_WANTED_POSE_CLIMB = new LoggedNetworkNumber("/Tuning/Climb/CLIMB_WANTED_POSE_CLIMB", ClimbConstants.CLIMB_WANTED_POSE_CLIMB);
    private static final LoggedNetworkNumber CLIMB_TRAVEL_POSE = new LoggedNetworkNumber("/Tuning/Climb/CLIMB_TRAVEL_POSE", ClimbConstants.CLIMB_TRAVEL_POSE);

    public static final double kS_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kS.get() : ClimbConstants.kS;
    }

    public static final double kV_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kV.get() : ClimbConstants.kV;
    }

    public static final double kA_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kA.get() : ClimbConstants.kA;
    }

    public static final double kP_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kP.get() : ClimbConstants.kP;
    }

    public static final double kI_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kI.get() : ClimbConstants.kI;
    }

    public static final double kD_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? kD.get() : ClimbConstants.kD;
    }

    public static final double MotionMagicCruiseVelocity_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? MotionMagicCruiseVelocity.get() : ClimbConstants.MotionMagicCruiseVelocity;
    }

    public static final double MotionMagicAcceleration_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? MotionMagicAcceleration.get() : ClimbConstants.MotionMagicAcceleration;
    }

    public static final double MotionMagicJerk_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? MotionMagicJerk.get() : ClimbConstants.MotionMagicJerk;
    }

    public static final double CLIMB_WANTED_POSE_CLIMB_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? CLIMB_WANTED_POSE_CLIMB.get() : ClimbConstants.CLIMB_WANTED_POSE_CLIMB;
    }

    public static final double CLIMB_TRAVEL_POSE_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? CLIMB_TRAVEL_POSE.get() : ClimbConstants.CLIMB_TRAVEL_POSE;
    }
}
