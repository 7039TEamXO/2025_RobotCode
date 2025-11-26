package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Operator;
import frc.robot.Constants.SysIdMode;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.IO.Real.CameraReal;
import frc.robot.subsystems.IO.Real.ClimbReal;
import frc.robot.subsystems.IO.Real.ElevatorReal;
import frc.robot.subsystems.IO.Real.HandlerReal;
import frc.robot.subsystems.IO.Real.TrayReal;
import frc.robot.subsystems.IO.Real.WristReal;
import frc.robot.subsystems.IO.Sim.CameraSim;
import frc.robot.subsystems.IO.Sim.ClimbSim;
import frc.robot.subsystems.IO.Sim.ElevatorSim;
import frc.robot.subsystems.IO.Sim.HandlerSim;
import frc.robot.subsystems.IO.Sim.TraySim;
import frc.robot.subsystems.IO.Sim.WristSim;
import frc.robot.subsystems.IO.Stub.CameraStub;
import frc.robot.subsystems.IO.Stub.ClimbStub;
import frc.robot.subsystems.IO.Stub.ElevatorStub;
import frc.robot.subsystems.IO.Stub.HandlerStub;
import frc.robot.subsystems.IO.Stub.TrayStub;
import frc.robot.subsystems.IO.Stub.WristStub;
import frc.robot.subsystems.Handler.Handler;
// import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
import frc.robot.subsystems.Tray.Tray;
import frc.robot.subsystems.Wrist.Wrist;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

// import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  private static Command driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
      () -> MathUtil.applyDeadband(-SubsystemManager.getPSJoystick().getLeftY(), Operator.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-SubsystemManager.getPSJoystick().getLeftX(), Operator.LEFT_X_DEADBAND),
      () -> -SubsystemManager.getPSJoystick().getRightX());
      
  public static Climb climb;
  public static Elevator elevator;
  public static Handler handler;
  public static Tray tray;
  public static Wrist wrist;

  public static void init() {
    switch(Constants.CurrentMode) {
      case REAL:
        climb = new Climb(new ClimbReal());
        elevator = new Elevator(new ElevatorReal());
        handler = new Handler(new HandlerReal());
        tray = new Tray(new TrayReal());
        wrist = new Wrist(new WristReal());
        Limelight.init(new CameraReal());
        break;
      case REPLAY: 
        climb = new Climb(new ClimbStub());
        elevator = new Elevator(new ElevatorStub());
        handler = new Handler(new HandlerStub());
        tray = new Tray(new TrayStub());
        wrist = new Wrist(new WristStub());
        Limelight.init(new CameraStub());
        break;
      case SIM:
        climb = new Climb(new ClimbSim(0.02));
        elevator = new Elevator(new ElevatorSim(0.02));
        handler = new Handler(new HandlerSim(0.02));
        tray = new Tray(new TraySim(0.02));
        wrist = new Wrist(new WristSim(0.02));
        Limelight.init(new CameraSim());
        break;
    }

    SubsystemManager.init();

    configureDriveCommand();
    configureBindings();
  }

  private static void configureBindings() {    
    SubsystemManager.getPSJoystick().PS().onTrue((Commands.runOnce(SubsystemManager.getDrivebase()::zeroGyroWithAlliance)));
      // to use in case the rotation is messed up

    if(Constants.CurrentSysIdMode == SysIdMode.ACTIVE) {
      SubsystemManager.getPSJoystick().L1().onTrue(Commands.runOnce(() -> SignalLogger.start()));
      SubsystemManager.getPSJoystick().R1().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

      // Elevator SysId [General subsystem template]

      SubsystemManager.getPSJoystick().circle().whileTrue(RobotContainer.elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      SubsystemManager.getPSJoystick().square().whileTrue(RobotContainer.elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      SubsystemManager.getPSJoystick().triangle().whileTrue(RobotContainer.elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
      SubsystemManager.getPSJoystick().cross().whileTrue(RobotContainer.elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  private static void configureDriveCommand() {
    if (teamColorIsBlue() == 0) {
      SubsystemManager.getDrivebase().zeroGyro();
      SubsystemManager.getDrivebase().resetOdometry(new Pose2d(SubsystemManager.getDrivebase().getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftY(), Constants.Operator.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftX(), Constants.Operator.LEFT_X_DEADBAND)),
        () -> modifyAxis(-SubsystemManager.getPSJoystick().getRightX(), Constants.Operator.RIGHT_X_DEADBAND));
    } else if(teamColorIsBlue() == 1) {
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftY(), Constants.Operator.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftX(), Constants.Operator.LEFT_X_DEADBAND)),
        () -> modifyAxis(-SubsystemManager.getPSJoystick().getRightX(), Constants.Operator.RIGHT_X_DEADBAND));
    }
    
    SubsystemManager.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  public static Command getAutonomousCommand() {
    return SubsystemManager.getDrivebase().getAutonomousCommand(Dashboard.getSelectedAutonomy());
  }

  public static void setDriveMode()
  {}

  public static void setMotorBrake(boolean brake) {
    SubsystemManager.getDrivebase().setMotorBrake(brake);
  }

  public static int teamColorIsBlue() {
    try {
      Optional<Alliance> color = DriverStation.getAlliance();
	    return color.get() == DriverStation.Alliance.Blue ? 1 : 0;
    } catch (Exception e) {
      return -1;
    }
  }

  private static double modifyAxis(double joystickInput, double deadband) {
    double deadbandValue = MathUtil.applyDeadband(joystickInput, deadband);
    double scaled_input = 1 - (1 - Math.abs(deadbandValue)) * (1 - 0.1);
    return deadbandValue * scaled_input;
  }
}
