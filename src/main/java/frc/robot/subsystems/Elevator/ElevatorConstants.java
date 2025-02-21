package frc.robot.subsystems.Elevator;

import frc.robot.Dashboard;

public class ElevatorConstants {
    public static final int ElevatorRightMotorID = 8;
    public static final int ElevatorLeftMotorID = 10;

    public static final double EncoderMultiplier = 1;

    public static final double kS = 0.5; // Add 0.25 V output to overcome static friction
    public static final double kV = 0; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 2.5; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0.7; // output for integrated error
    public static final double kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    
    public static final double MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
    public static final double MotionMagicAcceleration = 60; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)

    public static final double StatorCurrentLimit = 80;
    public static final double SupplyCurrentLimit = 60;

    public static final double ELEVATOR_POSE_BASE = 1; //0
    public static final double ELEVATOR_POSE_LEVEL0 = 1;
    public static final double ELEVATOR_POSE_LEVEL1 = 4.84;
    public static final double ELEVATOR_POSE_LEVEL2 = 10.57;
    public static final double ELEVATOR_POSE_LEVEL3 = 18.25;
    public static final double ELEVATOR_POSE_ALGAE_LOW = 6.7;//7
    public static final double ELEVATOR_POSE_ALGAE_HIGH = 11.9;//12.2

    public static final double ELEVATOR_POSE_SAFE_TO_ROTATE = 1;

    public static final double maxPos = 18.49;
}
