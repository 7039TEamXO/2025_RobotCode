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

    // DriveToPose PIDs (DTP-General)
    public static final double KP = 2.2; // 2.5
    public static final double KI = 0;
    public static final double KD = 0.1; // 0.2

    public static final double KP_ANGULAR = 3;
    public static final double KI_ANGULAR = 0;
    public static final double KD_ANGULAR = 0.1;

    // DTP-Feeder PIDs
    public static final double KP_FEEDER = 2.2;
    public static final double KI_FEEDER = 0;
    public static final double KD_FEEDER = 0.1;

    public static final double KP_FEEDER_ANGULAR = 3;
    public static final double KI_FEEDER_ANGULAR = 0;
    public static final double KD_FEEDER_ANGULAR = 0.1;

    // Align-at-reef PIDs
    public static final double KP_ALIGN = 1.4;
    public static final double KD_ALIGN = 0.15;

    // Used solely in avoidToPose
    public static final double FILTER_TIME_CONSTANT = 4;

    // PROXIMITY TOLERANCE
    public static final double CLOSE_DISTANCE_ERROR_CAP = 0.1;
    public static final double CLOSE_ANGLE_ERROR_CAP = Math.toRadians(5);

    public static final double FAR_DISTANCE = 1.7;
    public static final double VERY_FAR_DISTANCE = 1.8;
}