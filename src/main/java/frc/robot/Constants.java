// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
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

  public static final TuningMode CurrentTuningMode = TuningMode.TUNE;

  public static enum TuningMode {
    /** Use the constants from the Tuning panel. */
    TUNE,

    /** Ignore the tuned constants. */
    IGNORE
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; // 1s, 20ms + 110ms due to SPARK MAX velocity lag

  public static class OperatorConstants
  {
    // Joystick Deadbands
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 2; 
  }

  public static class ReefConstants {
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

  public static class FieldConstants {
    // NET LOCATIONS
    public static final double WANTED_X_NET_ALGAE_POS_BLUE = 7.25;
    public static final double WANTED_ROTATION_ANGLE_NET_ALGAE_POS_BLUE = 0;

    public static final double WANTED_X_NET_ALGAE_POS_RED = 10.6; //10.4
    public static final double WANTED_ROTATION_ANGLE_NET_ALGAE_POS_RED = 180;

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
