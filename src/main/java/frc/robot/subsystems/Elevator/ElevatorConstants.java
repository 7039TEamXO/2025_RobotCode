package frc.robot.subsystems.Elevator;

public class ElevatorConstants {
    public static final int ElevatorRightMotorID = 8;
    public static final int ElevatorLeftMotorID = 10;

    public static final double SysIdQuasistatic = 0.5;
    public static final double SysIdDynamic = 1.5;

    public static final double kS = 0.1;
    public static final double kV = 0.8 * 0.123;
    public static final double kA = 0.007;
    public static final double kP = 2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.57;

    public static final double MMExpo_CV = 0;
    public static final double MMExpo_kV = 0.1;
    public static final double MMExpo_kA = 0.11;

    public static final double StatorCurrentLimit = 80;
    public static final double SupplyCurrentLimit = 60;

    public static final double ELEVATOR_POSE_BASE = 0; //0
    public static final double ELEVATOR_POSE_INTAKE_CORAL = 0; //0.7
    public static final double ELEVATOR_POSE_LEVEL0 = 0.9; //0.7
    public static final double ELEVATOR_POSE_LEVEL1 = 5.1; //4.84
    public static final double ELEVATOR_POSE_LEVEL2 = 11.1; //10.57
    public static final double ELEVATOR_POSE_LEVEL3 = 18.7; //18.4
    public static final double ELEVATOR_POSE_ALGAE_LOW = 5.3;
    public static final double ELEVATOR_POSE_ALGAE_HIGH = 11; //11
    public static final double ELEVATOR_POSE_ALGAE_IN_OFFSET = 2.7;
    public static final double ELEVATOR_POSE_ALGAE_LOW_NET = 0.5; //0.3
    public static final double ELEVATOR_POSE_ALGAE_HIGH_NET = 6.3; //6.1
    public static final double ELEVATOR_POSE_ALGAE_THROW_POS = 13; 

    public static final double ELEVATOR_POSE_SAFE_TO_ROTATE = 1;

    public static final double EncoderMaxPos = 18.49;
}
