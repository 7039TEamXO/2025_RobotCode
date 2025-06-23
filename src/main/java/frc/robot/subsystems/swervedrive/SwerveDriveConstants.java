package frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.config.PIDConstants;

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

    // Autonomy PIDs
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1, 0.0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(2.5, 0, 0);

    // DriveToPose PIDs
    public static final double KP = 2;
    public static final double KD = 0.3;

    public static final double KP_ANGULAR = 2.7;
    public static final double KD_ANGULAR = 0;

    // PROXIMITY TOLERANCE
    public static final double CLOSE_DISTANCE_ERROR_CAP = 0.15;
    public static final double CLOSE_ANGLE_ERROR_CAP = 5;

    public static final double FAR_DISTANCE = 1.7;
    public static final double VERY_FAR_DISTANCE = 1.8;
}