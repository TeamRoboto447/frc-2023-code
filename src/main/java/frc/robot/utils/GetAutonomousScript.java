package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;

public class GetAutonomousScript {
    public static Trajectory getTrajectory(Script script, int step) {
        switch (script) {
            case SCORE_AND_CHARGE:
                return getScoreAndChargeStep(step);
            case DEMO_SCRIPT_1:
                return getDemo1Step(step);
            default:
                System.out.println("How did we get here?");
                return null;
        }
    }

    private static Trajectory getScoreAndChargeStep(int step) {
        switch(step) {
            default: return null;
        }
    }

    private static Trajectory getDemo1Step(int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(
                                new Translation2d(-0.5, 0.1)),
                        new Pose2d(-1, 0, Rotation2d.fromDegrees(90)),
                        AutoConstants.trajectoryConfig);
            case 2:
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(-1, 0, new Rotation2d(90)),
                        List.of(
                                new Translation2d(-1.1, -1)),
                        new Pose2d(-1, -2, Rotation2d.fromDegrees(-90)),
                        AutoConstants.trajectoryConfig);
            default: return null;
        }
    }

    public static enum Script {
        SCORE_AND_CHARGE,
        DEMO_SCRIPT_1
    }
}
