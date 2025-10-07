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
    private static final LoggedNetworkNumber KD = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) kD", SwerveDriveConstants.KD);
    private static final LoggedNetworkNumber KP_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kP", SwerveDriveConstants.KP_ANGULAR);
    private static final LoggedNetworkNumber KD_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kD", SwerveDriveConstants.KD_ANGULAR);

    private static final LoggedNetworkNumber KP_FEEDER = new LoggedNetworkNumber("/Tuning/Swerve/(DTP-Feeder) kP", SwerveDriveConstants.KP_FEEDER);
    private static final LoggedNetworkNumber KD_FEEDER = new LoggedNetworkNumber("/Tuning/Swerve/(DTP-Feeder) kD", SwerveDriveConstants.KD_FEEDER);
    private static final LoggedNetworkNumber KP_FEEDER_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP-Feeder) Angle kP", SwerveDriveConstants.KP_FEEDER_ANGULAR);
    private static final LoggedNetworkNumber KD_FEEDER_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP-Feeder) Angle kD", SwerveDriveConstants.KD_FEEDER_ANGULAR);

    private static final LoggedNetworkNumber KP_ALIGN = new LoggedNetworkNumber("/Tuning/Swerve/(Align) Angle kP", SwerveDriveConstants.KP_ALIGN);
    private static final LoggedNetworkNumber KD_ALIGN = new LoggedNetworkNumber("/Tuning/Swerve/(Align) Angle kD", SwerveDriveConstants.KD_ALIGN);

    private static final LoggedNetworkNumber FILTER_TIME_CONSTANT = new LoggedNetworkNumber("/Tuning/Swerve/Filter Time Constant", SwerveDriveConstants.FILTER_TIME_CONSTANT);

    private static final LoggedNetworkNumber CLOSE_DISTANCE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Close Distance Error Cap", SwerveDriveConstants.CLOSE_DISTANCE_ERROR_CAP);
    private static final LoggedNetworkNumber CLOSE_ANGLE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Close Angle Error Cap", SwerveDriveConstants.CLOSE_ANGLE_ERROR_CAP);
    private static final LoggedNetworkNumber FAR_DISTANCE = new LoggedNetworkNumber("/Tuning/Swerve/Far Distance", SwerveDriveConstants.FAR_DISTANCE);
    private static final LoggedNetworkNumber VERY_FAR_DISTANCE = new LoggedNetworkNumber("/Tuning/Swerve/Very Far Distance", SwerveDriveConstants.VERY_FAR_DISTANCE);

    private static final LoggedNetworkNumber TEST_DRIVE_X = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) X", 12.25);
    private static final LoggedNetworkNumber TEST_DRIVE_Y = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) Y", 5.1);
    private static final LoggedNetworkNumber TEST_DRIVE_ANGLE = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) Angle", -60);

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

    public static final double KD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD.get() : SwerveDriveConstants.KD;
    }

    public static final double KP_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP_ANGULAR.get() : SwerveDriveConstants.KP_ANGULAR;
    }

    public static final double KD_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD_ANGULAR.get() : SwerveDriveConstants.KD_ANGULAR;
    }

    public static final double KP_FEEDER_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP_FEEDER.get() : SwerveDriveConstants.KP_FEEDER;
    }

    public static final double KD_FEEDER_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD_FEEDER.get() : SwerveDriveConstants.KD_FEEDER;
    }

    public static final double KP_FEEDER_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP_FEEDER_ANGULAR.get() : SwerveDriveConstants.KP_FEEDER_ANGULAR;
    }

    public static final double KD_FEEDER_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD_FEEDER_ANGULAR.get() : SwerveDriveConstants.KD_FEEDER_ANGULAR;
    }

    public static final double KP_ALIGN_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP_ALIGN.get() : SwerveDriveConstants.KP_ALIGN;
    }

    public static final double KD_ALIGN_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD_ALIGN.get() : SwerveDriveConstants.KD_ALIGN;
    }

    public static final double FILTER_TIME_CONSTANT_get() { 
        return Constants.GetTuningMode() != TuningMode.IGNORE ? FILTER_TIME_CONSTANT.get() : SwerveDriveConstants.FILTER_TIME_CONSTANT;
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
