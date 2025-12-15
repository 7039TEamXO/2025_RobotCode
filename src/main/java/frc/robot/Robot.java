// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.RobotModel;
import frc.robot.subsystems.SubsystemManager;
import edu.wpi.first.net.WebServer;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
  private static Robot instance;

  private Command autonomousCommand;
  private Command autoCommand;
  private Command autoInitCommand;
  private boolean isFirstTimeAtDisabled = true;

  private static boolean onAuto;

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;

    // Set up data receivers & replay source
    switch (Constants.CurrentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new WPILOGWriter("logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
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
    // Pathfinding.setPathfinder(new LocalADStarAK());

    RobotContainer.init();
    Dashboard.init();
    // cameraSetup();
    LED.init();

    // Activate Elastic layout
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    // PathfindingCommand.warmupCommand().schedule();
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
    LED.setLedData();
    Dashboard.update();
    RobotModel.periodic();

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
      // autoInitCommand = new PathPlannerAuto(robotContainer.getAutonomousCommand()).ignoringDisable(true);
      // autoInitCommand.schedule();
      isFirstTimeAtDisabled = false;
      System.out.println("First time at disabled!");
    }

    RobotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    onAuto = false;

    RobotContainer.configureDriveCommand();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    RobotContainer.setMotorBrake(true);
    autonomousCommand = RobotContainer.getAutonomousCommand();

    // SubsystemManager.init();
    if (autonomousCommand != null) {
      autoCommand = new WaitCommand(0.01).andThen(autonomousCommand);
    }

    // schedule the autonomous command (example)
    
    if (autoCommand != null)
    {
      // m_autonomousCommand.schedule();
      autoCommand.schedule();
    }

    SubsystemManager.setStateToDefault();;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  { 
    onAuto = true;
    SubsystemManager.operate(true);
  }

  @Override
  public void teleopInit()
  {
    SubsystemManager.setStateToDefault();;

    if (autoInitCommand != null) {
      autoInitCommand.cancel();
    }
    
    if (autoCommand != null)
    {
      autoCommand.cancel();
    }
    RobotContainer.setDriveMode();
    RobotContainer.setMotorBrake(true);
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
  {}

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

  public static boolean isAuto() {
    return onAuto;
  }

  // LEGACY CODE
  // public void cameraSetup() {
  //   // USB CAMERA //
  //   try {
  //     CameraServer.startAutomaticCapture();
  //     // // Creates UsbCamera and MjpegServer [1] and connects them
  //     //   CameraServer.startAutomaticCapture();
  //     //   // Creates the CvSink and connects it to the UsbCamera
  //     //   CvSink cvSink = CameraServer.getVideo();
  //     //   // Creates the CvSource and MjpegServer [2] and connects them
  //     //   CvSource outputStream = CameraServer.putVideo("Blur", 480, 480);
  //   } catch (Exception e) {
  //     System.out.println("--------------- CameraSetup ERROR ---------------");
  //   }
  // }
}
