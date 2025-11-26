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

    private static final LoggedNetworkNumber MMEXpo_CV = new LoggedNetworkNumber("/Tuning/Elevator/(Motion Magic) CV", ElevatorConstants.MMExpo_CV);
    private static final LoggedNetworkNumber MMExpo_kV = new LoggedNetworkNumber("/Tuning/Elevator/(Motion Magic) kV", ElevatorConstants.MMExpo_kV);
    private static final LoggedNetworkNumber MMExpo_kA = new LoggedNetworkNumber("/Tuning/Elevator/(Motion Magic) kA", ElevatorConstants.MMExpo_kA);

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

    public static final double MMExpo_CV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MMEXpo_CV.get() : ElevatorConstants.MMExpo_CV;
    }

    public static final double MMExpo_kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MMExpo_kV.get() : ElevatorConstants.MMExpo_kV;
    }

    public static final double MMExpo_kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MMExpo_kA.get() : ElevatorConstants.MMExpo_kA;
    }
}
