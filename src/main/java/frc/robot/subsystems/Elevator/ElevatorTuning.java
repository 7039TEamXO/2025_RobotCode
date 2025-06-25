package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class ElevatorTuning {
    private static final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Elevator/kS", ElevatorConstants.kS);
    private static final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Elevator/kV", ElevatorConstants.kV);
    private static final LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Elevator/kA", ElevatorConstants.kA);
    private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Elevator/kP", ElevatorConstants.kP);
    private static final LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Elevator/kI", ElevatorConstants.kI);
    private static final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Elevator/kD", ElevatorConstants.kD);
    private static final LoggedNetworkNumber kG = new LoggedNetworkNumber("/Tuning/Elevator/kG", ElevatorConstants.kG);

    private static final LoggedNetworkNumber MotionMagicCruiseVelocity = new LoggedNetworkNumber("/Tuning/Elevator/(Motion Magic) Cruise Velocity", ElevatorConstants.MotionMagicCruiseVelocity);
    private static final LoggedNetworkNumber MotionMagicAcceleration = new LoggedNetworkNumber("/Tuning/Elevator/(Motion Magic) Acceleration", ElevatorConstants.MotionMagicAcceleration);
    private static final LoggedNetworkNumber MotionMagicJerk = new LoggedNetworkNumber("/Tuning/Elevator/(Motion Magic) Jerk", ElevatorConstants.MotionMagicJerk);

    public static final double kS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kS.get() : ElevatorConstants.kS;
    }

    public static final double kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kV.get() : ElevatorConstants.kV;
    }

    public static final double kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kA.get() : ElevatorConstants.kA;
    }

    public static final double kP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kP.get() : ElevatorConstants.kP;
    }

    public static final double kI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kI.get() : ElevatorConstants.kI;
    }

    public static final double kD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kD.get() : ElevatorConstants.kD;
    }

    public static final double kG_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kG.get() : ElevatorConstants.kG;
    }

    public static final double MotionMagicCruiseVelocity_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MotionMagicCruiseVelocity.get() : ElevatorConstants.MotionMagicCruiseVelocity;
    }

    public static final double MotionMagicAcceleration_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MotionMagicAcceleration.get() : ElevatorConstants.MotionMagicAcceleration;
    }

    public static final double MotionMagicJerk_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MotionMagicJerk.get() : ElevatorConstants.MotionMagicJerk;
    }
}
