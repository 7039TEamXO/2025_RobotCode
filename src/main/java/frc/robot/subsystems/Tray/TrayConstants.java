package frc.robot.subsystems.Tray;

public class TrayConstants {
    public static final int TrayMotorID = 5; // CHANGE [!]

    public static final double kS = 0.1; // Add 0.25 V output to overcome static friction
    public static final double kV = 0.05; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 2; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0.3; // no output for integrated error
    public static final double kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    public static final double MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    public static final double MotionMagicAcceleration = 10; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)

    public static final double StatorCurrentLimit = 30;
    public static final double SupplyCurrentLimit = 20;

    public static final double MaxEncoderPos = 13.77;

    public static final double TRAY_POS_BASE = -0.02;
    public static final double TRAY_POS_UP = 1.3; // CHANGE [!]
    public static final double TRAY_MAX_POSE = 1.55;
}
