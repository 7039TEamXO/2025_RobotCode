package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class SwerveDriveTuning {
    private static final LoggedNetworkNumber MIN_SPEED = new LoggedNetworkNumber("/Tuning/Swerve/MIN_SPEED", SwerveDriveConstants.MIN_SPEED);
    private static final LoggedNetworkNumber CLIMB_SPEED = new LoggedNetworkNumber("/Tuning/Swerve/CLIMB_SPEED", SwerveDriveConstants.CLIMB_SPEED);
    private static final LoggedNetworkNumber MAX_ROTATION_V = new LoggedNetworkNumber("/Tuning/Swerve/MAX_ROTATION_V", SwerveDriveConstants.MAX_ROTATION_V);
    private static final LoggedNetworkNumber MIN_ROTATION_V = new LoggedNetworkNumber("/Tuning/Swerve/MIN_SPEED", SwerveDriveConstants.MIN_ROTATION_V);
    
    private static final LoggedNetworkNumber KP = new LoggedNetworkNumber("/Tuning/Swerve/KP", SwerveDriveConstants.KP);
    private static final LoggedNetworkNumber KD = new LoggedNetworkNumber("/Tuning/Swerve/KD", SwerveDriveConstants.KD);
    private static final LoggedNetworkNumber KP_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/KP_ANGULAR", SwerveDriveConstants.KP_ANGULAR);
    private static final LoggedNetworkNumber KD_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/KD_ANGULAR", SwerveDriveConstants.KD_ANGULAR);

    private static final LoggedNetworkNumber CLOSE_DISTANCE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/CLOSE_DISTANCE_ERROR_CAP", SwerveDriveConstants.CLOSE_DISTANCE_ERROR_CAP);
    private static final LoggedNetworkNumber CLOSE_ANGLE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/CLOSE_ANGLE_ERROR_CAP", SwerveDriveConstants.CLOSE_ANGLE_ERROR_CAP);
    private static final LoggedNetworkNumber FAR_DISTANCE = new LoggedNetworkNumber("/Tuning/Swerve/FAR_DISTANCE", SwerveDriveConstants.FAR_DISTANCE);
    private static final LoggedNetworkNumber VERY_FAR_DISTANCE = new LoggedNetworkNumber("/Tuning/Swerve/VERY_FAR_DISTANCE", SwerveDriveConstants.VERY_FAR_DISTANCE);

    private static final LoggedNetworkNumber TEST_DRIVE_X = new LoggedNetworkNumber("/Tuning/Swerve/TEST_DRIVE_X", 0);
    private static final LoggedNetworkNumber TEST_DRIVE_Y = new LoggedNetworkNumber("/Tuning/Swerve/TEST_DRIVE_Y", 0);
    private static final LoggedNetworkNumber TEST_DRIVE_ANGLE = new LoggedNetworkNumber("/Tuning/Swerve/TEST_DRIVE_ANGLE", 0);

    public static final double MIN_SPEED_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? MIN_SPEED.get() : SwerveDriveConstants.MIN_SPEED;
    }

    public static final double CLIMB_SPEED_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? CLIMB_SPEED.get() : SwerveDriveConstants.CLIMB_SPEED;
    }

    public static final double MAX_ROTATION_V_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? MAX_ROTATION_V.get() : SwerveDriveConstants.MAX_ROTATION_V;
    }

    public static final double MIN_ROTATION_V_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? MIN_ROTATION_V.get() : SwerveDriveConstants.MIN_ROTATION_V;
    }

    public static final double KP_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? KP.get() : SwerveDriveConstants.KP;
    }

    public static final double KD_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? KD.get() : SwerveDriveConstants.KD;
    }

    public static final double KP_ANGULAR_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? KP_ANGULAR.get() : SwerveDriveConstants.KP_ANGULAR;
    }

    public static final double KD_ANGULAR_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? KD_ANGULAR.get() : SwerveDriveConstants.KD_ANGULAR;
    }

    public static final double CLOSE_DISTANCE_ERROR_CAP_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? CLOSE_DISTANCE_ERROR_CAP.get() : SwerveDriveConstants.CLOSE_DISTANCE_ERROR_CAP;
    }

    public static final double CLOSE_ANGLE_ERROR_CAP_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? CLOSE_ANGLE_ERROR_CAP.get() : SwerveDriveConstants.CLOSE_ANGLE_ERROR_CAP;
    }

    public static final double FAR_DISTANCE_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? FAR_DISTANCE.get() : SwerveDriveConstants.FAR_DISTANCE;
    }

    public static final double VERY_FAR_DISTANCE_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? VERY_FAR_DISTANCE.get() : SwerveDriveConstants.VERY_FAR_DISTANCE;
    }

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
