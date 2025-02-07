package frc.robot.subsystems.Elevator;

public class ElevatorConstants {
    public static final int ElevatorRightMotorID = 8;
    public static final int ElevatorLeftMotorID = 10;

    public static final double EncoderMultiplier = 1;

    public static final double kS = 0.5; // Add 0.25 V output to overcome static friction
    public static final double kV = 0; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 2.0; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0.75; // no output for integrated error
    public static final double kD = 0.01; // A velocity error of 1 rps results in 0.1 V output
    
    public static final double MotionMagicCruiseVelocity = 40; // Target cruise velocity of 80 rps
    public static final double MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    public static final double StatorCurrentLimit = 60;
    public static final double SupplyCurrentLimit = 30;

    public static final double ELEVATOR_POSE_BASE = 0;
    public static final double ELEVATOR_POSE_LEVEL0 = 1;
    public static final double ELEVATOR_POSE_LEVEL1 = 3.44;
    public static final double ELEVATOR_POSE_LEVEL2 = 9.17;
    public static final double ELEVATOR_POSE_LEVEL3 = 18.49;
    public static final double ELEVATOR_POSE_ALGAE_LOW = 6.0;
    public static final double ELEVATOR_POSE_ALGAE_HIGH = 12.3;

    public static final double maxPos = 18.49;
}
