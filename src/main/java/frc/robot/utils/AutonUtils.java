package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonUtils {
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

    public static Pose2d getStartingPose(Trajectory traj, DriveSubsystem driveSubsystem) {
        return new Pose2d(traj.getInitialPose().getX(), traj.getInitialPose().getY(), driveSubsystem.getPose().getRotation());
    }

    public static double rotationOffsetCorrection(double target) {
        return target + DriveConstants.angleOffset;
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
