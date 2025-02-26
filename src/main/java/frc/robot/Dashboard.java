package frc.robot;

// import java.util.logging.Handler;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.DeliveryManager;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Handler.Handler;
import frc.robot.subsystems.Tray.Tray;

public class Dashboard {
    private final static SendableChooser<String> m_chooser = new SendableChooser<>();
    private static String m_autoSelected;
    private static ShuffleboardTab driver = Shuffleboard.getTab("Driver");
    private static ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");
    private static ShuffleboardTab subsystemsInformation = Shuffleboard.getTab("SubsystemsInformation");
    private static ShuffleboardTab debugging = Shuffleboard.getTab("Debugging");
    private static HttpCamera limelightcamera = new HttpCamera("limelight", "http://10.70.39.11:5801");

    private static GenericEntry add_value_to_Wrist = subsystemsInformation.add("SetWristValue", 0).withPosition(3, 8).withSize(3, 3)
            .getEntry();
    private static GenericEntry add_value_to_Elevator = subsystemsInformation.add("SetElevatorValue", 0).withPosition(8, 8).withSize(3, 3)
            .getEntry();
    // private static GenericEntry restart_wrist = subsystemsInformation.add("restart_wrist", 0).withPosition(12, 8).withSize(3, 3)
    //         .getEntry().getBoolean(false);
    

    public static void init() {
        Autos[] states = Autos.values();
        for (int i = 0; i < states.length; i++) {
            Autos state = states[i];
            String name_state = state.getAutoName();
            if (i == 0) {
                m_chooser.setDefaultOption(name_state, name_state);
            }
            m_chooser.addOption(name_state, name_state);
        }  

        driver.add("Autos", m_chooser).withPosition(0, 0).withSize(5, 3);
        driver.add("Limelight Camera", limelightcamera).withPosition(17, 0).withSize(9, 6);

        telemetry.addNumber("X", () -> SubsystemManager.getDriveBase().getPose().getX()).withPosition(0, 0).withSize(8, 3);
        telemetry.addNumber("Y", () -> SubsystemManager.getDriveBase().getPose().getY()).withPosition(0, 3).withSize(8, 3);
        telemetry.addNumber("Rot", () -> SubsystemManager.getDriveBase().getPose().getRotation().getDegrees()).withPosition(0, 6).withSize(8, 3);
    
        telemetry.addNumber("Front Left Rot", () -> SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[0].getPosition().angle.getDegrees()).withPosition(9, 0).withSize(8, 3);
        telemetry.addNumber("Front Right Rot", () -> SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[1].getPosition().angle.getDegrees()).withPosition(17, 0).withSize(8, 3);
        telemetry.addNumber("Back Left Rot", () -> SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[2].getPosition().angle.getDegrees()).withPosition(9, 3).withSize(8, 3);
        telemetry.addNumber("Back Right Rot", () -> SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[3].getPosition().angle.getDegrees()).withPosition(17, 3).withSize(8, 3);
        
        subsystemsInformation.addNumber("Elevator_raw", () -> Elevator.getCurrentPosition()).withPosition(0, 0).withSize(3, 3);
        subsystemsInformation.addNumber("Wrist_raw", () -> Wrist.getCurrentPosition()).withPosition(3, 0).withSize(3, 3);
        subsystemsInformation.addNumber("Tray_raw", () -> Tray.getTrayPosition()).withPosition(0, 3).withSize(3, 3);
        subsystemsInformation.addNumber("Climb_raw", () -> Climb.getClimbPose()).withPosition(3, 3).withSize(3, 3);
        subsystemsInformation.addBoolean("CoralIR ", ()-> Handler.getCoralIr()).withPosition(6, 0).withSize(3, 3);
        subsystemsInformation.addNumber("AlgaeIR ", ()-> Handler.getAlgaeIrValue()).withPosition(9, 0).withSize(3, 3);

        subsystemsInformation.addString("ElevatorState",() -> SubsystemManager.getElevatorState().name()).withPosition(17, 0).withSize(5, 3);
        subsystemsInformation.addString("RobotState",() -> SubsystemManager.getRobotState().name()).withPosition(22, 0).withSize(3, 3);
        subsystemsInformation.addString("HandlerState",() -> SubsystemManager.getHandlerState().name()).withPosition(17, 3).withSize(3, 3);
        subsystemsInformation.addString("WristState", () -> DeliveryManager.getWristState().name()).withPosition(20, 3).withSize(3, 3);
        subsystemsInformation.addString("ClimbState", () -> SubsystemManager.getClimbState().name()).withPosition(23, 3).withSize(3, 3);

        subsystemsInformation.addBoolean("isCoralIn", () -> Handler.isCoralIn()).withPosition(6, 3).withSize(3, 3);
        subsystemsInformation.addBoolean("isAlgaelIn", () -> Handler.isAlgaeIn()).withPosition(9, 3).withSize(3, 3);

        debugging.addNumber("HandlerCounter", () -> Handler.getCoralIntakeCounter()).withPosition(0, 0).withSize(3,3);
        debugging.addBoolean("HandlerReset", () -> Handler.getReset()).withPosition(3, 0).withSize(3,3);

        debugging.addString("ElevatorState",() -> SubsystemManager.getElevatorState().name()).withPosition(17, 0).withSize(5, 3);
        debugging.addString("LastElevatorState",() -> SubsystemManager.getLastElevatorState().name()).withPosition(22, 0).withSize(5, 3);
    }

    public static String getSelectedAutonomy() {
        return m_autoSelected = m_chooser.getSelected();
    }

    // public static boolean isRestartWrist() {
    //     return restart_wrist;
    // }

    public static double add_value_to_Elevator() {
        return add_value_to_Elevator.getDouble(0);
    }

    public static double add_value_to_Wrist() {
        return add_value_to_Wrist.getDouble(0);
    }
}
