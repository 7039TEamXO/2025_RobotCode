// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final Mode CurrentMode = RobotBase.isReal() ? Mode.REAL : (RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms spark max velocity lag
        // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final double MAX_SPEED  = Units.feetToMeters(15); //v14.5
  public static final double MIN_SPEED  = Units.feetToMeters(4);

  public static final double CLIMB_SPEED = Units.feetToMeters(7);

  public static final double MAX_ROTATION_V = 4;
  public static final double MIN_ROTATION_V = 2;
  public static final double DEG_TO_RAD = Math.PI / 180;

  public static final class AutoConstants
  { 
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1, 0.0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(2.5, 0, 0);
  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 3; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 2; //6 
  }
}
