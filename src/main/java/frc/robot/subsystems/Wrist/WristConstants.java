package frc.robot.subsystems.Wrist;

public class WristConstants {
    public static final double kS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0; // no output for integrated error
    public static final double kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    public static final double MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    public static final double MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    public static final double StatorCurrentLimit = 50;
    public static final double SupplyCurrentLimit = 30;
}
