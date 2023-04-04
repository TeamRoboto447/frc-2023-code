package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.FollowTrajectoryDeadReckoning;
import frc.robot.commands.FollowTrajectoryOdometry;
import frc.robot.commands.LockRobot;
import frc.robot.commands.MoveArmToLimit;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.OpenGrabberForPickupStation;
import frc.robot.commands.SetGrabber;
import frc.robot.commands.SetGrabberExtension;
import frc.robot.commands.SetGrabberExtensionWithIntake;
import frc.robot.commands.SetGrabberWithIntake;
import frc.robot.commands.MoveArmToLimit.Limit;
import frc.robot.subsystems.DriveSubsystem;

public class AutonUtils {
    private static Trajectory getTrajectory(Script script, Pose2d startingPos, int step) {
        switch (script) {
            case SCORE_AND_CHARGE_LONG:
                return getScoreAndChargeLongStep(startingPos, step);
            case SCORE_AND_CHARGE_SHORT:
                return getScoreAndChargeShortStep(startingPos, step);
            case TAG_1_RED:
                return getTagOneStep(startingPos, step);
            case TAG_2_RED:
                return getTagTwoStep(startingPos, step);
            case TAG_3_RED:
                return getTagThreeStep(startingPos, step);
            case TAG_8_BLUE:
                return getTagSixStep(startingPos, step);
            case TAG_7_BLUE:
                return getTagSevenStep(startingPos, step);
            case TAG_6_BLUE:
                return getTagEightStep(startingPos, step);
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

    private static Trajectory getScoreAndChargeLongStep(Pose2d startingPose, int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(0), Units.feetToMeters(10.5), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            default:
                return null;
        }
    }

    private static Trajectory getScoreAndChargeShortStep(Pose2d startingPose, int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(0), Units.feetToMeters(8.5), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            default:
                return null;
        }
    }

    private static Trajectory getTagOneStep(Pose2d startingPose, int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(0), Units.feetToMeters(10.5), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            default:
                return null;
        }
    }

    private static Trajectory getTagTwoStep(Pose2d startingPose, int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(0), Units.feetToMeters(13.5), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            // case 2:
            // return TrajectoryGenerator.generateTrajectory(
            // startingPose,
            // List.of(),
            // new Pose2d(Units.feetToMeters(42.1), Units.feetToMeters(10),
            // startingPose.getRotation()),
            // AutoConstants.trajectoryConfig);

            default:
                return null;
        }
    }

    private static Trajectory getTagThreeStep(Pose2d startingPose, int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(38), Units.feetToMeters(15), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);

            default:
                return null;
        }
    }

    private static Trajectory getTagSixStep(Pose2d startingPose, int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(20), Units.feetToMeters(16), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            case 2:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(20), Units.feetToMeters(16), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);

            default:
                return null;
        }
    }

    private static Trajectory getTagSevenStep(Pose2d startingPose, int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(19), Units.feetToMeters(10), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            case 2:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(14.5), Units.feetToMeters(10), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);

            default:
                return null;
        }
    }

    private static Trajectory getTagEightStep(Pose2d startingPose, int step) {
        switch (step) {
            case 1:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(22), Units.feetToMeters(4), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            case 2:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(22), Units.feetToMeters(4), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);

            default:
                return null;
        }
    }

    public static Command getCommandScript(RobotContainer container, Script script) {
        Pose2d startingPose = container.m_robotDrive.getPose();
        switch (script) {
            case SCORE_AND_CHARGE_LONG:
                Trajectory score_and_charge_long_traj1 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        1); // Get first movement trajectory
                FollowTrajectoryDeadReckoning score_and_charge_long_movement1 = new FollowTrajectoryDeadReckoning(
                        container.m_robotDrive,
                        score_and_charge_long_traj1,
                        startingPose.getRotation(),
                        false); // Create a new movement command for the first movement

                return new SequentialCommandGroup( // This runs the movements in order
                        new InstantCommand(
                                () -> container.m_robotDrive.resetOdometry(
                                        AutonUtils.getStartingPose(
                                                score_and_charge_long_traj1,
                                                container.m_robotDrive))), // Ensure the robot is where it thinks it is
                        // if dead
                        // reckoning

                        new SetGrabberExtension(container.m_robotArm, true),
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, 1),
                        new SetGrabberExtension(container.m_robotArm, true),
                        score_and_charge_long_movement1,
                        new LockRobot(container.m_robotArm, container.m_robotDrive));
            case SCORE_AND_CHARGE_SHORT:
                Trajectory score_and_charge_short_traj1 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        1); // Get first movement trajectory
                FollowTrajectoryDeadReckoning score_and_charge_short_movement1 = new FollowTrajectoryDeadReckoning(
                        container.m_robotDrive,
                        score_and_charge_short_traj1,
                        startingPose.getRotation(),
                        false); // Create a new movement command for the first movement

                return new SequentialCommandGroup( // This runs the movements in order
                        new InstantCommand(
                                () -> container.m_robotDrive.resetOdometry(
                                        AutonUtils.getStartingPose(
                                                score_and_charge_short_traj1,
                                                container.m_robotDrive))), // Ensure the robot is where it thinks it is
                        // if dead
                        // reckoning

                        new SetGrabberExtension(container.m_robotArm, true),
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, 1),
                        new SetGrabberExtension(container.m_robotArm, true),
                        score_and_charge_short_movement1,
                        new LockRobot(container.m_robotArm, container.m_robotDrive));

            case TAG_8_BLUE:
                return new SequentialCommandGroup( // This runs the movements in o\rder
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, -0.1, 0.2),
                        // new MoveArmToLimit(container.m_robotArm, Limit.TOP_VERTICAL, Limit.NO_CHANGE,
                        // 0),
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, 1, 1),
                        new SetGrabber(container.m_robotArm, true, 0.5),
                        new WaitCommand(0.25),
                        new MoveArmToLimit(container.m_robotArm, Limit.NO_CHANGE, Limit.CLOSE_HORIZONTAL, 0),
                        new SetGrabberExtension(container.m_robotArm, true),
                        new AutoBalance(container.m_robotDrive, container.m_robotArm));
            case TAG_7_BLUE:
                startingPose = container.m_robotDrive.getPose(); // Setup starting pose
                Trajectory tag_7_traj1 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        1); // Get first movement trajectory
                FollowTrajectoryOdometry tag_7_movement1 = new FollowTrajectoryOdometry(
                        container.m_robotDrive,
                        tag_7_traj1,
                        startingPose.getRotation(),
                        false); // Create a new movement command for the first movement

                startingPose = new Pose2d(
                        Units.feetToMeters(19),
                        Units.feetToMeters(10),
                        startingPose.getRotation()); // Update starting pose for next movement

                Trajectory tag_7_traj2 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        2); // Get second movement trajectory

                FollowTrajectoryOdometry tag_7_movement2 = new FollowTrajectoryOdometry(
                        container.m_robotDrive,
                        tag_7_traj2,
                        startingPose.getRotation(),
                        true); // Create a new movement command for the second movement

                return new SequentialCommandGroup( // This runs the movements in order
                        new InstantCommand(
                                () -> container.m_robotDrive.resetOdometry(
                                        AutonUtils.getStartingPose(
                                                tag_7_traj1,
                                                container.m_robotDrive))), // Ensure the robot is where it thinks it is
                        // if dead
                        // reckoning

                        new SetGrabberExtension(container.m_robotArm, true),
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, 1),
                        new SetGrabberExtension(container.m_robotArm, true),

                        // new SetGrabberExtension(container.m_robotArm, false),
                        // new SetGrabber(container.m_robotArm, false),
                        // new MoveArmToPosition(container.m_robotArm, Double.NaN,0, Double.NaN), // NaN
                        // = don't move
                        // new MoveArmToPosition(container.m_robotArm, 0, Double.NaN, Double.NaN), //
                        // NaN = don't move

                        tag_7_movement1, // Do First Movement
                        tag_7_movement2, // Do second Movement
                        new InstantCommand(
                                () -> container.m_robotDrive.stopModules())); // Ensure Robot Is Stopped;

            case TAG_6_BLUE:

                startingPose = container.m_robotDrive.getPose(); // Setup starting pose
                Trajectory tag_6_traj1 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        1); // Get first movement trajectory
                FollowTrajectoryOdometry tag_6_movement1 = new FollowTrajectoryOdometry(
                        container.m_robotDrive,
                        tag_6_traj1,
                        startingPose.getRotation(),
                        true); // Create a new movement command for the first movement

                // startingPose = new Pose2d(
                // Units.feetToMeters(20),
                // Units.feetToMeters(14),
                // startingPose.getRotation()); // Update starting pose for next movement

                // Trajectory tag_6_traj2 = AutonUtils.getTrajectory(
                // script,
                // startingPose,
                // 2); // Get second movement trajectory

                // FollowTrajectory tag_6_movement2 = new FollowTrajectory(
                // container.m_robotDrive,
                // tag_6_traj2,
                // startingPose.getRotation(),
                // true); // Create a new movement command for the second movement

                return new SequentialCommandGroup( // This runs the movements in order
                        new InstantCommand(
                                () -> container.m_robotDrive.resetOdometry(
                                        AutonUtils.getStartingPose(
                                                tag_6_traj1,
                                                container.m_robotDrive))), // Ensure the robot is where it thinks it is
                        // if dead
                        // reckoning

                        new SetGrabberExtension(container.m_robotArm, true),
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, 1),
                        new SetGrabberExtension(container.m_robotArm, true),

                        tag_6_movement1, // Do First Movement
                        // tag_6_movement2, // Do second Movement
                        new InstantCommand(
                                () -> container.m_robotDrive.stopModules())); // Ensure Robot Is Stopped;
            case TAG_1_RED:

                startingPose = container.m_robotDrive.getPose(); // Setup starting pose
                Trajectory tag_1_traj1 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        1); // Get first movement trajectory
                FollowTrajectoryDeadReckoning tag_1_movement1 = new FollowTrajectoryDeadReckoning(
                        container.m_robotDrive,
                        tag_1_traj1,
                        startingPose.getRotation(),
                        true); // Create a new movement command for the first movement

                // startingPose = new Pose2d(
                // Units.feetToMeters(33),
                // Units.feetToMeters(8),
                // startingPose.getRotation()); // Update starting pose for next movement

                // Trajectory tag_1_traj2 = AutonUtils.getTrajectory(
                // script,
                // startingPose,
                // 2); // Get second movement trajectory

                // FollowTrajectory tag_1_movement2 = new FollowTrajectory(
                // container.m_robotDrive,
                // tag_1_traj2,
                // startingPose.getRotation(),
                // true); // Create a new movement command for the second movement

                return new SequentialCommandGroup( // This runs the movements in order
                        new InstantCommand(
                                () -> container.m_robotDrive.resetOdometry(
                                        AutonUtils.getStartingPose(
                                                tag_1_traj1,
                                                container.m_robotDrive))), // Ensure the robot is where it thinks it is
                        // if dead
                        // reckoning

                        new SetGrabberExtension(container.m_robotArm, true),
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, 1),
                        new SetGrabberExtension(container.m_robotArm, true),

                        // new SetGrabberExtension(container.m_robotArm, false),
                        // new SetGrabber(container.m_robotArm, false),
                        // new MoveArmToPosition(container.m_robotArm, Double.NaN,0, Double.NaN), // NaN
                        // = don't move
                        // new MoveArmToPosition(container.m_robotArm, 0, Double.NaN, Double.NaN), //
                        // NaN = don't move

                        tag_1_movement1, // Do First Movement
                        // tag_1_movement2, // Do second Movement
                        new LockRobot(container.m_robotArm, container.m_robotDrive)); // Ensure Robot Is Stopped;

            case TAG_2_RED:
                startingPose = container.m_robotDrive.getPose(); // Setup starting pose
                Trajectory tag_2_traj1 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        1); // Get first movement trajectory
                FollowTrajectoryDeadReckoning tag_2_movement1 = new FollowTrajectoryDeadReckoning(
                        container.m_robotDrive,
                        tag_2_traj1,
                        startingPose.getRotation(),
                        true); // Create a new movement command for the first movement

                // startingPose = new Pose2d(
                // Units.feetToMeters(38.6),
                // Units.feetToMeters(10),
                // startingPose.getRotation()); // Update starting pose for next movement

                // Trajectory tag_2_traj2 = AutonUtils.getTrajectory(
                // script,
                // startingPose,
                // 2); // Get second movement trajectory

                // FollowTrajectoryOdometry tag_2_movement2 = new FollowTrajectoryOdometry(
                // container.m_robotDrive,
                // tag_2_traj2,
                // startingPose.getRotation(),
                // true); // Create a new movement command for the second movement

                return new SequentialCommandGroup( // This runs the movements in order
                        new InstantCommand(
                                () -> container.m_robotDrive.resetOdometry(
                                        AutonUtils.getStartingPose(
                                                tag_2_traj1,
                                                container.m_robotDrive))), // Ensure the robot is where it thinks it is
                        // if dead
                        // reckoning

                        new SetGrabberExtension(container.m_robotArm, true),
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, 1),
                        new SetGrabberExtension(container.m_robotArm, true),

                        // new SetGrabberExtension(container.m_robotArm, false),
                        // new SetGrabber(container.m_robotArm, false),
                        // new MoveArmToPosition(container.m_robotArm, Double.NaN,0, Double.NaN), // NaN
                        // = don't move
                        // new MoveArmToPosition(container.m_robotArm, 0, Double.NaN, Double.NaN), //
                        // NaN = don't move

                        tag_2_movement1, // Do First Movement
                        // tag_2_movement2, // Do second Movement
                        new LockRobot(container.m_robotArm, container.m_robotDrive)); // Ensure Robot Is Stopped;

            case TAG_3_RED:
                return new SequentialCommandGroup( // This runs the movements in o\rder
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, -0.1, 0.2),
                        // new MoveArmToLimit(container.m_robotArm, Limit.TOP_VERTICAL, Limit.NO_CHANGE,
                        // 0),
                        new SetGrabberExtensionWithIntake(container.m_robotArm, true, 1, 1),
                        new SetGrabber(container.m_robotArm, true, 0.5),
                        new WaitCommand(0.25),
                        new MoveArmToLimit(container.m_robotArm, Limit.NO_CHANGE, Limit.CLOSE_HORIZONTAL, 0),
                        new InstantCommand(
                                () -> container.m_robotDrive.stopModules())); // Ensure Robot Is Stopped;
            case BALANCE:
                return new SequentialCommandGroup(
                        new SetGrabberExtension(container.m_robotArm, true, 0),
                        // new SetGrabberExtensionWithIntake(container.m_robotArm, true, 1),
                        // new MoveArmToLimit(container.m_robotArm, Limit.BOTTOM_VERTICAL,
                        // Limit.CLOSE_HORIZONTAL, 0),
                        new AutoBalance(container.m_robotDrive, container.m_robotArm));
            default:
                return new SequentialCommandGroup();
        }
    }

    public static enum Script {
        SCORE_AND_CHARGE_LONG,
        SCORE_AND_CHARGE_SHORT,
        TAG_8_BLUE,
        TAG_7_BLUE,
        TAG_6_BLUE,
        TAG_1_RED,
        TAG_2_RED,
        TAG_3_RED,
        BALANCE
    }
}
