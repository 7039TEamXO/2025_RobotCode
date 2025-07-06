package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Handler.Handler;

public class RedRightCoralAuto extends SequentialCommandGroup {
    public final Pose2d START;
    public final Pose2d REEF_1;
    public final Pose2d REEF_2;
    public final Pose2d REEF_3;
    public final Pose2d REEF_4;
    public final Pose2d FEEDER;

    public RedRightCoralAuto() {
        START = new Pose2d(10.135, 6, new Rotation2d());
        REEF_1 = new Pose2d(12.53, 5.27, new Rotation2d(Math.toRadians(-60)));
        REEF_2 = new Pose2d(13.59, 5.27, new Rotation2d(Math.toRadians(-120)));
        REEF_3 = new Pose2d(13.87, 5.10, new Rotation2d(Math.toRadians(-120)));
        REEF_4 = new Pose2d(14.40, 4.19, new Rotation2d(Math.toRadians(-180)));
        FEEDER = new Pose2d(16.20, 7.30, new Rotation2d(Math.toRadians(-125)));

        setup();
    }

    public RedRightCoralAuto(Pose2d _START, Pose2d _REEF_1, Pose2d _REEF_2, Pose2d _REEF_3, Pose2d _REEF_4, Pose2d _FEEDER) {
        START = _START;
        REEF_1 = _REEF_1;
        REEF_2 = _REEF_2;
        REEF_3 = _REEF_3;
        REEF_4 = _REEF_4;
        FEEDER = _FEEDER;

        setup();
    }

    private void setup() {
        addCommands(
            new InstantCommand(() -> {
                SubsystemManager.getDrivebase().resetOdometry(START);
            }),
            SubsystemManager.operateAuto(RobotState.TRAVEL, ElevatorState.BASE),
            new ParallelRaceGroup(
                SubsystemManager.getDrivebase().avoidToPose(REEF_1),
                new SequentialCommandGroup(
                    SubsystemManager.operateAuto(RobotState.TRAVEL, ElevatorState.LEVEL3).repeatedly()
                        .until(() -> SubsystemManager.getDrivebase().isCloseEnoughToPose(REEF_1)),
                    Commands.waitSeconds(0.3),
                    SubsystemManager.operateAuto(RobotState.DEPLETE, null),
                    Commands.waitSeconds(0.3)
                )
            ),

            SubsystemManager.getDrivebase().avoidToPose(FEEDER)
                .until(() -> SubsystemManager.getDrivebase().isCloseEnoughToPose(FEEDER)),
            new ParallelRaceGroup(
                SubsystemManager.getDrivebase().avoidToPose(FEEDER),
                Commands.waitSeconds(0.3)
            ),
            new ParallelRaceGroup(
                SubsystemManager.getDrivebase().avoidToPose(REEF_2),
                new SequentialCommandGroup(
                    SubsystemManager.operateAuto(RobotState.TRAVEL, ElevatorState.LEVEL3)
                        .onlyIf(() -> Handler.isCoralIn()).repeatedly()
                        .until(() -> SubsystemManager.getDrivebase().isCloseEnoughToPose(REEF_2)),
                    Commands.waitSeconds(0.3),
                    SubsystemManager.operateAuto(RobotState.DEPLETE, null),
                    Commands.waitSeconds(0.3)
                )
            ),

            SubsystemManager.getDrivebase().avoidToPose(FEEDER)
                .until(() -> SubsystemManager.getDrivebase().isCloseEnoughToPose(FEEDER)),
            new ParallelRaceGroup(
                SubsystemManager.getDrivebase().avoidToPose(FEEDER),
                Commands.waitSeconds(0.3)
            ),
            new ParallelRaceGroup(
                SubsystemManager.getDrivebase().avoidToPose(REEF_3),
                new SequentialCommandGroup(
                    SubsystemManager.operateAuto(RobotState.TRAVEL, ElevatorState.LEVEL3)
                        .onlyIf(() -> Handler.isCoralIn()).repeatedly()
                        .until(() -> SubsystemManager.getDrivebase().isCloseEnoughToPose(REEF_3)),
                    Commands.waitSeconds(0.3),
                    SubsystemManager.operateAuto(RobotState.DEPLETE, null),
                    Commands.waitSeconds(0.3)
                )
            ),

            SubsystemManager.getDrivebase().avoidToPose(FEEDER)
                .until(() -> SubsystemManager.getDrivebase().isCloseEnoughToPose(FEEDER)),
            new ParallelRaceGroup(
                SubsystemManager.getDrivebase().avoidToPose(FEEDER),
                Commands.waitSeconds(0.3)
            ),
            new ParallelRaceGroup(
                SubsystemManager.getDrivebase().avoidToPose(REEF_4),
                new SequentialCommandGroup(
                    SubsystemManager.operateAuto(RobotState.TRAVEL, ElevatorState.LEVEL3)
                        .onlyIf(() -> Handler.isCoralIn()).repeatedly()
                        .until(() -> SubsystemManager.getDrivebase().isCloseEnoughToPose(REEF_4)),
                    Commands.waitSeconds(0.3),
                    SubsystemManager.operateAuto(RobotState.DEPLETE, null),
                    Commands.waitSeconds(0.3)
                )
            ),

            SubsystemManager.getDrivebase().avoidToPose(FEEDER)
        );
    }
}
