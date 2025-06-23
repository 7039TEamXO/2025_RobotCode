package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class SwerveDriveTuning {
    private static final LoggedNetworkNumber MIN_SPEED = new LoggedNetworkNumber("/Tuning/Swerve/Minimum Speed", SwerveDriveConstants.MIN_SPEED);
    private static final LoggedNetworkNumber CLIMB_SPEED = new LoggedNetworkNumber("/Tuning/Swerve/Climb Speed", SwerveDriveConstants.CLIMB_SPEED);
    private static final LoggedNetworkNumber MAX_ROTATION_V = new LoggedNetworkNumber("/Tuning/Swerve/Maximum Rotation Velocity", SwerveDriveConstants.MAX_ROTATION_V);
    private static final LoggedNetworkNumber MIN_ROTATION_V = new LoggedNetworkNumber("/Tuning/Swerve/Minimum Rotation Velocity", SwerveDriveConstants.MIN_ROTATION_V);
    
    private static final LoggedNetworkNumber KP = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) kP", SwerveDriveConstants.KP);
    private static final LoggedNetworkNumber KI = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) kI", SwerveDriveConstants.KI);
    private static final LoggedNetworkNumber KD = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) kD", SwerveDriveConstants.KD);
    private static final LoggedNetworkNumber KP_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kP", SwerveDriveConstants.KP_ANGULAR);
    private static final LoggedNetworkNumber KI_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kI", SwerveDriveConstants.KI_ANGULAR);
    private static final LoggedNetworkNumber KD_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kD", SwerveDriveConstants.KD_ANGULAR);

    private static final LoggedNetworkNumber AUTO_KP = new LoggedNetworkNumber("/Tuning/Swerve/(Auto) kP", SwerveDriveConstants.AUTO_KP);
    private static final LoggedNetworkNumber AUTO_KI = new LoggedNetworkNumber("/Tuning/Swerve/(Auto) kI", SwerveDriveConstants.AUTO_KI);
    private static final LoggedNetworkNumber AUTO_KD = new LoggedNetworkNumber("/Tuning/Swerve/(Auto) kD", SwerveDriveConstants.AUTO_KD);
    private static final LoggedNetworkNumber AUTO_KP_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(Auto) Angle kP", SwerveDriveConstants.AUTO_KP_ANGULAR);
    private static final LoggedNetworkNumber AUTO_KI_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(Auto) Angle kI", SwerveDriveConstants.AUTO_KI_ANGULAR);
    private static final LoggedNetworkNumber AUTO_KD_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(Auto) Angle kD", SwerveDriveConstants.AUTO_KD_ANGULAR);

    private static final LoggedNetworkNumber CLOSE_DISTANCE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Close Distance Error Cap", SwerveDriveConstants.CLOSE_DISTANCE_ERROR_CAP);
    private static final LoggedNetworkNumber CLOSE_ANGLE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Close Angle Error Cap", SwerveDriveConstants.CLOSE_ANGLE_ERROR_CAP);
    private static final LoggedNetworkNumber FAR_DISTANCE = new LoggedNetworkNumber("/Tuning/Swerve/Far Distance", SwerveDriveConstants.FAR_DISTANCE);
    private static final LoggedNetworkNumber VERY_FAR_DISTANCE = new LoggedNetworkNumber("/Tuning/Swerve/Very Far Distance", SwerveDriveConstants.VERY_FAR_DISTANCE);

    private static final LoggedNetworkNumber TEST_DRIVE_X = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) X", 0);
    private static final LoggedNetworkNumber TEST_DRIVE_Y = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) Y", 0);
    private static final LoggedNetworkNumber TEST_DRIVE_ANGLE = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) Angle", 0);

    public static final double MIN_SPEED_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MIN_SPEED.get() : SwerveDriveConstants.MIN_SPEED;
    }

    public static final double CLIMB_SPEED_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? CLIMB_SPEED.get() : SwerveDriveConstants.CLIMB_SPEED;
    }

    public static final double MAX_ROTATION_V_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MAX_ROTATION_V.get() : SwerveDriveConstants.MAX_ROTATION_V;
    }

    public static final double MIN_ROTATION_V_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MIN_ROTATION_V.get() : SwerveDriveConstants.MIN_ROTATION_V;
    }

    public static final double KP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP.get() : SwerveDriveConstants.KP;
    }

    public static final double KI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KI.get() : SwerveDriveConstants.KI;
    }

    public static final double KD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD.get() : SwerveDriveConstants.KD;
    }

    public static final double KP_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP_ANGULAR.get() : SwerveDriveConstants.KP_ANGULAR;
    }

    public static final double KI_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KI_ANGULAR.get() : SwerveDriveConstants.KI_ANGULAR;
    }

    public static final double KD_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD_ANGULAR.get() : SwerveDriveConstants.KD_ANGULAR;
    }

    public static final double AUTO_KP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? AUTO_KP.get() : SwerveDriveConstants.AUTO_KP;
    }

    public static final double AUTO_KI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? AUTO_KI.get() : SwerveDriveConstants.AUTO_KI;
    }

    public static final double AUTO_KD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? AUTO_KD.get() : SwerveDriveConstants.AUTO_KD;
    }

    public static final double AUTO_KP_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? AUTO_KP_ANGULAR.get() : SwerveDriveConstants.AUTO_KP_ANGULAR;
    }

    public static final double AUTO_KI_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? AUTO_KI_ANGULAR.get() : SwerveDriveConstants.AUTO_KI_ANGULAR;
    }

    public static final double AUTO_KD_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? AUTO_KD_ANGULAR.get() : SwerveDriveConstants.AUTO_KD_ANGULAR;
    }

    public static final double CLOSE_DISTANCE_ERROR_CAP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? CLOSE_DISTANCE_ERROR_CAP.get() : SwerveDriveConstants.CLOSE_DISTANCE_ERROR_CAP;
    }

    public static final double CLOSE_ANGLE_ERROR_CAP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? CLOSE_ANGLE_ERROR_CAP.get() : SwerveDriveConstants.CLOSE_ANGLE_ERROR_CAP;
    }

    public static final double FAR_DISTANCE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? FAR_DISTANCE.get() : SwerveDriveConstants.FAR_DISTANCE;
    }

    public static final double VERY_FAR_DISTANCE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? VERY_FAR_DISTANCE.get() : SwerveDriveConstants.VERY_FAR_DISTANCE;
    }

    // Activates on an L3 press
    public static final double TEST_DRIVE_X_get() {
        return TEST_DRIVE_X.get();
    }

    public static final double TEST_DRIVE_Y_get() {
        return TEST_DRIVE_Y.get();
    }

    public static final double TEST_DRIVE_ANGLE_get() {
        return TEST_DRIVE_ANGLE.get();
    }
}
