package frc.robot.subsystems.Climb;

public class ClimbConstants {
    public static final int ClimbMotorID = 3;

    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;
    public static final double kP = 4.8;
    public static final double kI = 0;
    public static final double kD = 0.1;

    public static final double MotionMagicCruiseVelocity = 190;
    public static final double MotionMagicAcceleration = 80;
    public static final double MotionMagicJerk = 800;

    public static final double StatorCurrentLimit = 100;
    public static final double SupplyCurrentLimit = 100;

    public static final double CLIMB_WANTED_POWER_CLIMB = 1;
    public static final double CLIMB_WANTED_POWER_STOP = 0;
    public static final double CLIMB_WANTED_POWER_DESCEND = -0.8;
    public static final double CLIMB_WANTED_POSE_CLIMB = 235;
    public static final double CLIMB_TRAVEL_POSE = -34.5;

    public static final double EncoderMaxPos = 13.77;
}
