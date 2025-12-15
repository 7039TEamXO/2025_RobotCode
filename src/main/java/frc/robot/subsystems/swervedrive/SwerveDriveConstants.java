package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.util.Units;

/*
      -----------DRIVER STATION-----------

 *                  FACE ONE
 *               -----------
 *              /             \ 
 *   FACE SIX  /               \  FACE TWO
 *            /                 \
 *            \                 /
 *  FACE FIVE  \               /  FACE THREE
 *              \             /
 *                -----------
 *                 FACE FOUR
 *       
 */                   
public class SwerveDriveConstants {
    // SPEED LIMITATIONS
    public static final double MAX_SPEED = 4.5; // NOT TUNEABLE [!]
    public static final double MAX_ACCEL = 2.5;

    public static final double MIN_SPEED  = Units.feetToMeters(4); // Non-restrictive
    public static final double CLIMB_SPEED = Units.feetToMeters(7);

    public static final double MAX_ROTATION_V = 4;
    public static final double MIN_ROTATION_V = 2; // Non-restrictive

    // SysId
    public static final double DriveSysIdQuasistatic = 1;
    public static final double DriveSysIdDynamic = 7;
    public static final double TurnSysIdQuasistatic = 1;
    public static final double TurnSysIdDynamic = 7;

    // DriveToPose PIDs (DTP-General)
    public static final double KP = 2; // 2.5
    public static final double KP_X = 3;
    public static final double KI = 0;
    public static final double KD = 0; // 0.2

    public static final double KP_ANGULAR = 3;
    public static final double KI_ANGULAR = 0;
    public static final double KD_ANGULAR = 0;

    // PROXIMITY TOLERANCE
    public static final double CLOSE_DISTANCE_ERROR_CAP = 0.1;
    public static final double CLOSE_ANGLE_ERROR_CAP = Math.toRadians(5);

    // ERROR TOLERANCE
    public static final double MAXIMUM_DISTANCE_ERROR = 0.5;
    // public static final double MAXIMUM_ANGLE_ERROR = 10;

    public static final double FAR_DISTANCE = 1.7;
    public static final double VERY_FAR_DISTANCE = 1.8;
}