package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.util.Units;

/*
      -----------DRIVER STATION-----------

 *                  FACE ONE
 *                -----------
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
    public static final double MAX_SPEED  = Units.feetToMeters(15); // NOT TUNEABLE [!]

    public static final double MIN_SPEED  = Units.feetToMeters(4); // Non-restrictive
    public static final double CLIMB_SPEED = Units.feetToMeters(7);

    public static final double MAX_ROTATION_V = 4;
    public static final double MIN_ROTATION_V = 2; // Non-restrictive

    // Autonomy PIDs (unused)
    public static final double AUTO_KP = 1;
    public static final double AUTO_KI = 0;
    public static final double AUTO_KD = 0;

    public static final double AUTO_KP_ANGULAR = 2.5;
    public static final double AUTO_KI_ANGULAR = 0;
    public static final double AUTO_KD_ANGULAR = 0;

    // DriveToPose PIDs
    public static final double KP = 6;
    public static final double KI = 0;
    public static final double KD = 1;

    public static final double KP_ANGULAR = 5;
    public static final double KI_ANGULAR = 0;
    public static final double KD_ANGULAR = 0;

    // Used solely in avoidToPose
    public static final double FILTER_TIME_CONSTANT = 4;

    // PROXIMITY TOLERANCE
    public static final double CLOSE_DISTANCE_ERROR_CAP = 0.1;
    public static final double CLOSE_ANGLE_ERROR_CAP = Math.toRadians(5);

    public static final double FAR_DISTANCE = 1.7;
    public static final double VERY_FAR_DISTANCE = 1.8;
}