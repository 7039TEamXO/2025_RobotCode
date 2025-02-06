package frc.robot.subsystems.Elevator;

public class ElevatorConstants {
    public static final int ElevatorRightMotorID = 8;
    public static final int ElevatorLeftMotorID = 10;

    public static final double EncoderMultiplier = 1;

    public static final double kS = 0.8; // Add 0.25 V output to overcome static friction
    public static final double kV = 0; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 1.4; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0; // no output for integrated error
    public static final double kD = 0.04; // A velocity error of 1 rps results in 0.1 V output
    
    public static final double MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    public static final double MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    public static final double StatorCurrentLimit = 40;
    public static final double SupplyCurrentLimit = 20;

    public static final double maxPos = 18.49;
}
