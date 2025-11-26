// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;

public final class Constants
{
  public static final Mode CurrentMode = RobotBase.isReal() ? Mode.REAL : (RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY);
  // public static final Mode CurrentMode = Mode.REPLAY;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  private static final TuningMode StartingTuningMode = TuningMode.INACTIVE;
  private static final LoggedNetworkBoolean IsActiveTuning = 
    new LoggedNetworkBoolean("/Tuning/[!] Is Active Tuning", StartingTuningMode == TuningMode.ACTIVE ? true : false);

  public static TuningMode GetTuningMode() {
    return StartingTuningMode == TuningMode.IGNORE ? TuningMode.IGNORE : IsActiveTuning.get() ? TuningMode.ACTIVE : TuningMode.INACTIVE;
  }

  public static enum TuningMode {
    /** Use the constants from the Tuning panel, update. */
    ACTIVE,

    /** Use the constants from the Tuning panel, don't update. */
    INACTIVE,

    /** Ignore the tuned constants. */
    IGNORE
  }

  public static final SysIdMode CurrentSysIdMode = SysIdMode.INACTIVE;

  public static enum SysIdMode {
    ACTIVE, INACTIVE
  }

  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; // 100ms due to SPARK MAX velocity lag + 20ms loop time + 10ms for safety

  public static class Limelight
  {
    public static final Transform3d RobotToCamera = new Transform3d(new Translation3d(0.4, 0.0, 0.2), new Rotation3d(0, Math.toRadians(-20), 0));
    public static class CameraProperties {
      public static final int resWidth = 1280;
      public static final int resHeight = 960;
      public static final Rotation2d fovDiag = Rotation2d.fromDegrees(100);
      public static final double avgErrorPx = 0.007;
      public static final double errorStdDevPx = 0.0005;
      public static final double fps = 30;
      public static final double avgLatencyMs = 35;
      public static final double latencyStdDevMs = 5;

      public static final Matrix<N3, N1> singleTagStdDevs =  VecBuilder.fill(0.3, 0.3, 4);
      public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 2);
    }
  }

  public static class Operator
  {
    // Joystick Deadbands
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    // public static final double TURN_CONSTANT    = 2; 
  }

  public static class Reef {
    public static final double M_FROM_TAG_TO_POLES = 0.165;
    public static final double M_FROM_TAG_TO_ROBOT = 0.51;

    // REEF CENTER
    public static final double REEF_CENTER_X_RED = 13.055;
    public static final double REEF_CENTER_Y_RED = 4.025;
    public static final double REEF_CENTER_X_BLUE = 4.485;
    public static final double REEF_CENTER_Y_BLUE = 4.025;

    // REEF APRILTAGS
    public static final int BLUE_REEF_POS_FACE_ONE_ID   = 18;
    public static final int BLUE_REEF_POS_FACE_TWO_ID   = 19;
    public static final int BLUE_REEF_POS_FACE_THREE_ID = 20;
    public static final int BLUE_REEF_POS_FACE_FOUR_ID  = 21;
    public static final int BLUE_REEF_POS_FACE_FIVE_ID  = 22;
    public static final int BLUE_REEF_POS_FACE_SIX_ID   = 17;
    public static final int[] BLUE_REEF_TAGS_ARRAY = {BLUE_REEF_POS_FACE_ONE_ID,
                                                    BLUE_REEF_POS_FACE_TWO_ID,
                                                    BLUE_REEF_POS_FACE_THREE_ID,
                                                    BLUE_REEF_POS_FACE_FOUR_ID,
                                                    BLUE_REEF_POS_FACE_FIVE_ID,
                                                    BLUE_REEF_POS_FACE_SIX_ID};        

    public static final int RED_REEF_POS_FACE_ONE_ID    = 7;
    public static final int RED_REEF_POS_FACE_TWO_ID    = 6;
    public static final int RED_REEF_POS_FACE_THREE_ID  = 11;
    public static final int RED_REEF_POS_FACE_FOUR_ID   = 10;
    public static final int RED_REEF_POS_FACE_FIVE_ID   = 9;
    public static final int RED_REEF_POS_FACE_SIX_ID    = 8;       
    public static final int[] RED_REEF_TAGS_ARRAY = {RED_REEF_POS_FACE_ONE_ID,
                                                    RED_REEF_POS_FACE_TWO_ID,
                                                    RED_REEF_POS_FACE_THREE_ID,
                                                    RED_REEF_POS_FACE_FOUR_ID,
                                                    RED_REEF_POS_FACE_FIVE_ID,
                                                    RED_REEF_POS_FACE_SIX_ID};
  }

  public static class Field {
    // NET LOCATIONS
    public static final double WANTED_X_NET_ALGAE_POS_BLUE = 7.25;
    public static final double WANTED_Y_NET_ALGAE_POS_BLUE = 6;
    public static final double WANTED_ROTATION_ANGLE_NET_ALGAE_POS_BLUE = 0;

    public static final double WANTED_X_NET_ALGAE_POS_RED = 10.6; //10.4
    public static final double WANTED_Y_NET_ALGAE_POS_RED = 2; //10.4
    public static final double WANTED_ROTATION_ANGLE_NET_ALGAE_POS_RED = 180;

    // PROCESSOR LOCATIONS
    public static final double WANTED_X_PROCESSOR_ALGAE_POS_BLUE = 6;
    public static final double WANTED_Y_PROCESSOR_ALGAE_POS_BLUE = 0.2;
    public static final double WANTED_ROTATION_ANGLE_PROCESSOR_ALGAE_POS_BLUE = -90;

    public static final double WANTED_X_PROCESSOR_ALGAE_POS_RED = 11.5; //10.4
    public static final double WANTED_Y_PROCESSOR_ALGAE_POS_RED = 7.8;
    public static final double WANTED_ROTATION_ANGLE_PROCESSOR_ALGAE_POS_RED = 90;

    // FEEDER LOCATIONS
    public static final double WANTED_X_FEEDER_LEFT_RED = 16.5; // 16.241
    public static final double WANTED_Y_FEEDER_LEFT_RED = 0.8; // 0.614
    public static final double WANTED_ROTATION_ANGLE_FEEDER_LEFT_RED = 125;

    public static final double WANTED_X_FEEDER_RIGHT_RED = 16.2; // 16.155
    public static final double WANTED_Y_FEEDER_RIGHT_RED = 7.5; // 7.415
    public static final double WANTED_ROTATION_ANGLE_FEEDER_RIGHT_RED = -125;

    // COULD USE SOME MODIFICATION
    public static final double WANTED_X_FEEDER_LEFT_BLUE = 1.287;
    public static final double WANTED_Y_FEEDER_LEFT_BLUE = 7.286;
    public static final double WANTED_ROTATION_ANGLE_FEEDER_LEFT_BLUE = -55;

    public static final double WANTED_X_FEEDER_RIGHT_BLUE = 1.201;
    public static final double WANTED_Y_FEEDER_RIGHT_BLUE = 0.764;
    public static final double WANTED_ROTATION_ANGLE_FEEDER_RIGHT_BLUE = 55;
  }
}
