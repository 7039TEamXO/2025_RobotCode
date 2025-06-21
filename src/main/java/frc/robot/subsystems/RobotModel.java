package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.IO.Sim.ElevatorSim.ElevatorSimConstants;
import frc.robot.subsystems.Wrist.Wrist;

public class RobotModel {
    public class RobotModelConstants {
        private static final double FrameWidth = 0.725 * 2;
        
        private static final double BaseHeight = 0.05;
        private static final double SecondElevatorShift = 0.02;
        private static final double FrameUpwardShift = 0.06;
        private static final double FrameTotalShift = SecondElevatorShift + FrameUpwardShift;

        private static final double ElevatorShift = 0.11;
        private static final double WristShift = 0.2296 + ElevatorShift;

        public static final double WristRealGearRatio = 18.569;

        private static final double WristRootElevation = 0.5124;
        private static final double WristBaseLength = 0.2053;
        private static final double WristBackLength = 0.07575;

        private static final double WristBackLigamentLength = 0.22962;
        private static final double WristFrontLigamentLength = 0.26264;

        private static final double InitialWristAngle = -124.275;
        private static final double InitialWristBackAngle = InitialWristAngle + 180;
        private static final double InitialWristFrontLigamentAngle = InitialWristAngle + 19.42;
        private static final double InitialWristBackLigamentAngle = InitialWristAngle - 19.85;

        private static final double WristCoralLigamentLength = 0.1683;
        private static final double InitialWristCoralLigamentAngle = -120.283;
    }

    private static LoggedMechanism2d m_elevator =
        new LoggedMechanism2d(RobotModelConstants.FrameWidth, ElevatorSimConstants.MaxHeight + 2 * RobotModelConstants.WristRootElevation);

    private static LoggedMechanismRoot2d elevator_root =
        m_elevator.getRoot(
            "elevator_root",
            RobotModelConstants.FrameWidth / 2.0 + RobotModelConstants.ElevatorShift,
            0);

    private static LoggedMechanismLigament2d elevator =
        elevator_root.append(
            new LoggedMechanismLigament2d(
                "elevator", RobotModelConstants.BaseHeight + RobotModelConstants.SecondElevatorShift, 90, 10, new Color8Bit(Color.kBlue)));    
    
    private static LoggedMechanismLigament2d elevator_top =
        elevator.append(
            new LoggedMechanismLigament2d(
                "elevator_top", 1, 0, 5, new Color8Bit(Color.kGreen)));   

    private static LoggedMechanism2d m_wrist =
        new LoggedMechanism2d(RobotModelConstants.FrameWidth, ElevatorSimConstants.MaxHeight + 2 * RobotModelConstants.WristRootElevation);
        
    private static LoggedMechanismRoot2d wrist_root = 
        m_wrist.getRoot(
            "wrist_root",
            RobotModelConstants.FrameWidth / 2.0 + RobotModelConstants.WristShift,
            RobotModelConstants.BaseHeight + RobotModelConstants.WristRootElevation);

    private static LoggedMechanismLigament2d wrist_base =
        wrist_root.append(
            new LoggedMechanismLigament2d(
                "wrist_base", RobotModelConstants.WristBaseLength, RobotModelConstants.InitialWristAngle, 0, new Color8Bit(Color.kGreen)));

    private static LoggedMechanismLigament2d wrist_back =
        wrist_root.append(
            new LoggedMechanismLigament2d(
                "wrist_back", RobotModelConstants.WristBackLength, RobotModelConstants.InitialWristBackAngle, 0, new Color8Bit(Color.kGreen)));

    private static LoggedMechanismRoot2d wrist_backroot = 
        m_wrist.getRoot(
            "wrist_backroot",
            RobotModelConstants.FrameWidth / 2.0 + RobotModelConstants.WristShift + 
                Math.cos(Rotation2d.fromDegrees(RobotModelConstants.InitialWristBackAngle).getRadians()) * RobotModelConstants.WristBackLength,
            RobotModelConstants.BaseHeight + RobotModelConstants.WristRootElevation + 
                Math.sin(Rotation2d.fromDegrees(RobotModelConstants.InitialWristBackAngle).getRadians()) * RobotModelConstants.WristBackLength);
            
    private static LoggedMechanismLigament2d wrist_back_ligament = 
        wrist_backroot.append(
            new LoggedMechanismLigament2d("wrist_back_ligament", RobotModelConstants.WristBackLigamentLength, RobotModelConstants.InitialWristBackLigamentAngle, 2, new Color8Bit(Color.kRed)));

    private static LoggedMechanismLigament2d wrist_front_ligament = 
        wrist_backroot.append(
            new LoggedMechanismLigament2d("wrist_front_ligament", RobotModelConstants.WristFrontLigamentLength, RobotModelConstants.InitialWristFrontLigamentAngle, 2, new Color8Bit(Color.kRed)));

    @SuppressWarnings("unused")
    private static LoggedMechanismLigament2d wrist_coral_ligament = 
        wrist_front_ligament.append(
            new LoggedMechanismLigament2d("wrist_coral_ligament", RobotModelConstants.WristCoralLigamentLength, RobotModelConstants.InitialWristCoralLigamentAngle, 2, new Color8Bit(Color.kRed)));
    

    public static void periodic() {
        double cleanShift = Elevator.getCurrentPosition() * ElevatorSimConstants.MetersPerRotation;
        double framePosition = cleanShift + RobotModelConstants.BaseHeight + RobotModelConstants.FrameTotalShift;

        elevator.setLength(RobotModelConstants.BaseHeight + RobotModelConstants.SecondElevatorShift + cleanShift / 2.0);
        elevator_top.setLength(RobotModelConstants.FrameUpwardShift + cleanShift / 2.0);
        
        wrist_base.setAngle(Rotation2d.fromRotations(Wrist.getCurrentPosition() / RobotModelConstants.WristRealGearRatio).plus(Rotation2d.fromDegrees(RobotModelConstants.InitialWristAngle)));
        wrist_back.setAngle(Rotation2d.fromRotations(Wrist.getCurrentPosition() / RobotModelConstants.WristRealGearRatio).plus(Rotation2d.fromDegrees(RobotModelConstants.InitialWristBackAngle)));
        
        wrist_back_ligament.setAngle(Rotation2d.fromRotations(Wrist.getCurrentPosition() / RobotModelConstants.WristRealGearRatio).plus(Rotation2d.fromDegrees(RobotModelConstants.InitialWristBackLigamentAngle)));
        wrist_front_ligament.setAngle(Rotation2d.fromRotations(Wrist.getCurrentPosition() / RobotModelConstants.WristRealGearRatio).plus(Rotation2d.fromDegrees(RobotModelConstants.InitialWristFrontLigamentAngle)));
        double angle = Rotation2d.fromDegrees(wrist_back.getAngle()).getRadians();

        wrist_root.setPosition(RobotModelConstants.FrameWidth / 2.0 + RobotModelConstants.WristShift, framePosition + RobotModelConstants.WristRootElevation);
        wrist_backroot.setPosition(RobotModelConstants.FrameWidth / 2.0 + RobotModelConstants.WristShift + Math.cos(angle) * RobotModelConstants.WristBackLength, 
            framePosition + RobotModelConstants.WristRootElevation + Math.sin(angle) * RobotModelConstants.WristBackLength);

        Logger.recordOutput("Elevator", m_elevator);
        Logger.recordOutput("Wrist", m_wrist);
    }
}
