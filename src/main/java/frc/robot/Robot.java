// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Handler.Handler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.commands.PathPlannerAuto;

//import org.photonvision.estimation.VisionEstimation;

//import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  private static Robot   instance;
  private Command m_autonomousCommand;
  private Command autoCommand;
  private Command autoInitCommand;


  private boolean isFirstTimeAtDisabled = true;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    SubsystemManager.init();
    // shuflboard
    Dashboard.init();
    // Dashboard.setElevatorState();
    // Dashboard.setWristState();
    // Dashboard.acceptChanges();
    // Dashboard.cameraInit();
    cameraSetup();
    Limelight.init();
    LED.init();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // System.out.println(RobotContainer.teamColorIsBlue());
    LED.setLedData();
    Limelight.update();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {

    if (isFirstTimeAtDisabled) {
      autoInitCommand = new PathPlannerAuto(m_robotContainer.getAutonomousCommand()).ignoringDisable(true);
      autoInitCommand.schedule();
      isFirstTimeAtDisabled = false;
      System.out.println("first time at disabled");
    }

    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {

    SubsystemManager.getDriveBase().isAuto = false;

    // if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    // {
    //   m_robotContainer.setMotorBrake(false);
    //   disabledTimer.stop();
    // }

    // Limelight.printRobotPose();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {

    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // SubsystemManager.init();
    if (m_autonomousCommand != null) {
      autoCommand = new WaitCommand(0.01).andThen(m_autonomousCommand);
    }

    // schedule the autonomous command (example)
    
    if (autoCommand != null)
    {
      // m_autonomousCommand.schedule();
      autoCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  { 
    SubsystemManager.getDriveBase().isAuto = true;
    SubsystemManager.operate(true);
  }

  @Override
  public void teleopInit()
  {

    SubsystemManager.setState(RobotState.TRAVEL);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (autoInitCommand != null) {
      autoInitCommand.cancel();
    }
    
    if (autoCommand != null)
    {
      autoCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    SubsystemManager.operate(false);
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    // try
    // {
    //   new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    // } catch (IOException e)
    // {
    //   throw new RuntimeException(e);
    // }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
  public void cameraSetup() {
    // USB CAMERA //
    try {
      CameraServer.startAutomaticCapture();
    //   // Creates UsbCamera and MjpegServer [1] and connects them
    //     CameraServer.startAutomaticCapture();
    //     // Creates the CvSink and connects it to the UsbCamera
    //     CvSink cvSink = CameraServer.getVideo();
    //     // Creates the CvSource and MjpegServer [2] and connects them
    //     CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
    } catch (Exception e) {
      System.out.println("--------------- CameraSetup ERROR ---------------");
    }
  }

  public static CvSink getVideo() {
    return CameraServer.getVideo();
  }
}
