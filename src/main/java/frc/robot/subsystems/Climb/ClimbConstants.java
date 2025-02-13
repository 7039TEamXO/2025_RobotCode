package frc.robot.subsystems.Climb;

public class ClimbConstants {
    public static final int ClimbMotorID = 3;
    public static final int ServoMotorID = 5;

    public static final double CLIMB_WANTED_POWER_UP = 0.5;
    public static final double CLIMB_WANTED_POWER_STOP = 0;
    public static final double CLIMB_WANTED_POWER_DOWN = -0.5;

    public static final double kS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0; // no output for integrated error
    public static final double kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    public static final double MotionMagicCruiseVelocity = 190; // Target cruise velocity of 80 rps
    public static final double MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)

    public static final double StatorCurrentLimit = 30;
    public static final double SupplyCurrentLimit = 20;

    public static final double MaxEncoderPos = 13.77;
}
