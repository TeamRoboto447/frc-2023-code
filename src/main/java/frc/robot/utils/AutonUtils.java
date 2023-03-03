package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.SetGrabber;
import frc.robot.commands.SetGrabberExtension;
import frc.robot.subsystems.DriveSubsystem;

public class AutonUtils {
    private static Trajectory getTrajectory(Script script, Pose2d startingPos, int step) {
        switch (script) {
            case SCORE_AND_CHARGE:
                return getScoreAndChargeStep(startingPos, step);
            case LEAVE_COMMUNITY_AND_CHARGE:
                return getLeaveCommunityAndChargeStep(startingPos, step);
            default:
                System.out.println("How did we get here?");
                return null;
        }
    }

    private static Pose2d getStartingPose(Trajectory traj, DriveSubsystem driveSubsystem) {
        return new Pose2d(traj.getInitialPose().getX(), traj.getInitialPose().getY(),
                driveSubsystem.getPose().getRotation());
    }

    public static double rotationOffsetCorrection(double target) {
        return target + DriveConstants.angleOffset;
    }

    private static Trajectory getScoreAndChargeStep(Pose2d startingPos, int step) {
        switch (step) {
            default:
                return null;
        }
    }

    private static Trajectory getLeaveCommunityAndChargeStep(Pose2d startingPose, int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(26), Units.feetToMeters(2), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            case 2:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(33.5), Units.feetToMeters(9.5), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            default:
                return null;
        }
    }

    public static Command getCommandScript(RobotContainer container, Script script) {
        Pose2d startingPose = container.m_robotDrive.getPose(); // Setup starting pose
        Trajectory traj1 = AutonUtils.getTrajectory(
                Script.LEAVE_COMMUNITY_AND_CHARGE,
                startingPose,
                1); // Get first movement trajectory
        FollowTrajectory movement1 = new FollowTrajectory(
                container.m_robotDrive,
                traj1,
                startingPose.getRotation(),
                false); // Create a new movement command for the first movement

        startingPose = new Pose2d(
                Units.feetToMeters(26),
                Units.feetToMeters(9.5),
                startingPose.getRotation()); // Update starting pose for next movement

        Trajectory traj2 = AutonUtils.getTrajectory(
                Script.LEAVE_COMMUNITY_AND_CHARGE,
                startingPose,
                2); // Get second movement trajectory

        FollowTrajectory movement2 = new FollowTrajectory(
                container.m_robotDrive,
                traj2,
                startingPose.getRotation(),
                true); // Create a new movement command for the second movement

        return new SequentialCommandGroup( // This runs the movements in order
                new InstantCommand(
                        () -> container.m_robotDrive.resetOdometry(
                                AutonUtils.getStartingPose(
                                        traj1,
                                        container.m_robotDrive))), // Ensure the robot is where it thinks it is if dead
                                                                   // reckoning

                // new SetGrabberExtension(container.m_robotArm, true),
                // new MoveArmToPosition(container.m_robotArm, 12,Double.NaN, Double.NaN), //
                // Double.NaN = don't move (vert, horz, intake)
                //new SetGrabberExtension(container.m_robotArm, true),
                //new MoveArmToPosition(container.m_robotArm, Double.NaN, 50, Double.NaN), // Double.NaN = don't move
                                                                                         // (vert, horz, intake)
                //new SetGrabber(container.m_robotArm, true),

                //new SetGrabberExtension(container.m_robotArm, false),
                //new SetGrabber(container.m_robotArm, false),
                //new MoveArmToPosition(container.m_robotArm, Double.NaN,0, Double.NaN), // NaN
                // = don't move
                // new MoveArmToPosition(container.m_robotArm, 0, Double.NaN, Double.NaN), //
                // NaN = don't move

                 movement1, // Do First Movement
                 movement2, // Do second Movement
                new InstantCommand(
                        () -> container.m_robotDrive.stopModules())); // Endure Robot Is Stopped
    }

    public static enum Script {
        SCORE_AND_CHARGE,
        LEAVE_COMMUNITY_AND_CHARGE
    }
}
