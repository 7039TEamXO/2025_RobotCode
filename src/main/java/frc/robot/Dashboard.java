package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.Elevator;


public class Dashboard {
    private final static SendableChooser<String> m_chooser = new SendableChooser<>();
    private static String m_autoSelected;
    private static ShuffleboardTab driver = Shuffleboard.getTab("Driver");
    private static ShuffleboardTab subsystemsInformation = Shuffleboard.getTab("SubsystemsInformation");
    private static ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");
    private static HttpCamera limelightcamera = new HttpCamera("limelight", "http://10.70.39.11:5800");

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

        // SendableChooser<Double> m_robotPose = new SendableChooser<Double>();
        // m_robotPose.addOption("Robot Pose [X]", SubsystemManager.getDriveBase().getPose().getX());
        // m_robotPose.addOption("Robot Pose [Y]", SubsystemManager.getDriveBase().getPose().getY());
        // m_robotPose.addOption("Robot Pose [vRot]", SubsystemManager.getDriveBase().getPose().getRotation().getDegrees());

        driver.add("Autos", m_chooser).withPosition(0, 0).withSize(5, 3);
        driver.add("Limelight Camera", limelightcamera).withPosition(17, 0).withSize(9, 6);


        subsystemsInformation.addNumber("Elevatot raw encoder", () -> Elevator.getCurrentPosition()).withPosition(0, 0).withSize(5, 3);
        // telemetry.add("Robot Pose", m_robotPose).withPosition(0, 0).withSize(5, 3);

        telemetry.addNumber("X", () -> SubsystemManager.getDriveBase().getPose().getX()).withPosition(0, 0).withSize(8, 3);
        telemetry.addNumber("Y", () -> SubsystemManager.getDriveBase().getPose().getY()).withPosition(0, 3).withSize(8, 3);
        telemetry.addNumber("vRot", () -> SubsystemManager.getDriveBase().getPose().getRotation().getDegrees()).withPosition(0, 6).withSize(8, 3);
    
        telemetry.addNumber("Front Left vRot", () -> SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[0].getPosition().angle.getDegrees()).withPosition(9, 0).withSize(8, 3);
        telemetry.addNumber("Front Right vRot", () -> SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[1].getPosition().angle.getDegrees()).withPosition(17, 0).withSize(8, 3);
        telemetry.addNumber("Back Left vRot", () -> SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[2].getPosition().angle.getDegrees()).withPosition(9, 3).withSize(8, 3);
        telemetry.addNumber("Back Right vRot", () -> SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[3].getPosition().angle.getDegrees()).withPosition(17, 3).withSize(8, 3);
    }

    public static Autos getSelected() {
        if (m_chooser.getSelected() != null) {
            m_autoSelected = m_chooser.getSelected();
        }
        return Autos.valueOf(m_autoSelected);
    }
}
