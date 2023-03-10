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
import frc.robot.commands.SetGrabberExtensionWithIntake;
import frc.robot.subsystems.DriveSubsystem;

public class AutonUtils {
    private static Trajectory getTrajectory(Script script, Pose2d startingPos, int step) {
        switch (script) {
            case SCORE_AND_CHARGE:
                return getScoreAndChargeStep(startingPos, step);
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

    private static Trajectory getScoreAndChargeStep(Pose2d startingPose, int step) {
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
                        new Pose2d(Units.feetToMeters(14.5), Units.feetToMeters(9), startingPose.getRotation()),
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
                        new Pose2d(Units.feetToMeters(33), Units.feetToMeters(8), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
            case 2:
                return TrajectoryGenerator.generateTrajectory(
                        startingPose,
                        List.of(),
                        new Pose2d(Units.feetToMeters(33), Units.feetToMeters(8), startingPose.getRotation()),
                        AutoConstants.trajectoryConfig);
                default:
                return null;
        }
    }

    private static Trajectory getTagTwoStep(Pose2d startingPose, int step) {
        switch (step) {
            default:
                return null;
        }
    }

    private static Trajectory getTagThreeStep(Pose2d startingPose, int step) {
        switch (step) {
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
            case SCORE_AND_CHARGE:
                Trajectory score_and_charge_traj1 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        1); // Get first movement trajectory
                FollowTrajectory score_and_charge_movement1 = new FollowTrajectory(
                        container.m_robotDrive,
                        score_and_charge_traj1,
                        startingPose.getRotation(),
                        false); // Create a new movement command for the first movement

                startingPose = new Pose2d(
                        Units.feetToMeters(19),
                        Units.feetToMeters(10),
                        startingPose.getRotation()); // Update starting pose for next movement

                Trajectory score_and_charge_traj2 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        2); // Get second movement trajectory

                FollowTrajectory score_and_charge_movement2 = new FollowTrajectory(
                        container.m_robotDrive,
                        score_and_charge_traj2,
                        startingPose.getRotation(),
                        true); // Create a new movement command for the second movement

                return new SequentialCommandGroup( // This runs the movements in order
                        new InstantCommand(
                                () -> container.m_robotDrive.resetOdometry(
                                        AutonUtils.getStartingPose(
                                            score_and_charge_traj1,
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

                        // score_and_charge_movement1, // Do First Movement
                        // score_and_charge_movement2, // Do second Movement
                        new InstantCommand(
                                () -> container.m_robotDrive.stopModules())); // Ensure Robot Is Stopped;

            case TAG_8_BLUE:
            
            startingPose = container.m_robotDrive.getPose(); // Setup starting pose
            Trajectory tag_8_traj1 = AutonUtils.getTrajectory(
                    script,
                    startingPose,
                    1); // Get first movement trajectory
            FollowTrajectory tag_8_movement1 = new FollowTrajectory(
                    container.m_robotDrive,
                    tag_8_traj1,
                    startingPose.getRotation(),
                    true); // Create a new movement command for the first movement

            startingPose = new Pose2d(
                    Units.feetToMeters(22),
                    Units.feetToMeters(4),
                    startingPose.getRotation()); // Update starting pose for next movement

            Trajectory tag_8_traj2 = AutonUtils.getTrajectory(
                    script,
                    startingPose,
                    2); // Get second movement trajectory

            FollowTrajectory tag_8_movement2 = new FollowTrajectory(
                    container.m_robotDrive,
                    tag_8_traj2,
                    startingPose.getRotation(),
                    true); // Create a new movement command for the second movement

            return new SequentialCommandGroup( // This runs the movements in order
                    new InstantCommand(
                            () -> container.m_robotDrive.resetOdometry(
                                    AutonUtils.getStartingPose(
                                        tag_8_traj1,
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

                    tag_8_movement1, // Do First Movement
                   // tag_8_movement2, // Do second Movement
                    new InstantCommand(
                            () -> container.m_robotDrive.stopModules())); // Ensure Robot Is Stopped;
            case TAG_7_BLUE:
                startingPose = container.m_robotDrive.getPose(); // Setup starting pose
                Trajectory tag_7_traj1 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        1); // Get first movement trajectory
                FollowTrajectory tag_7_movement1 = new FollowTrajectory(
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

                FollowTrajectory tag_7_movement2 = new FollowTrajectory(
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
            FollowTrajectory tag_6_movement1 = new FollowTrajectory(
                    container.m_robotDrive,
                    tag_6_traj1,
                    startingPose.getRotation(),
                    true); // Create a new movement command for the first movement

            startingPose = new Pose2d(
                    Units.feetToMeters(20),
                    Units.feetToMeters(14),
                    startingPose.getRotation()); // Update starting pose for next movement

            Trajectory tag_6_traj2 = AutonUtils.getTrajectory(
                    script,
                    startingPose,
                    2); // Get second movement trajectory

            FollowTrajectory tag_6_movement2 = new FollowTrajectory(
                 container.m_robotDrive,
                    tag_6_traj2,
                    startingPose.getRotation(),
                    true); // Create a new movement command for the second movement

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
                FollowTrajectory tag_1_movement1 = new FollowTrajectory(
                        container.m_robotDrive,
                        tag_1_traj1,
                        startingPose.getRotation(),
                        true); // Create a new movement command for the first movement

                startingPose = new Pose2d(
                        Units.feetToMeters(33),
                        Units.feetToMeters(8),
                        startingPose.getRotation()); // Update starting pose for next movement

                Trajectory tag_1_traj2 = AutonUtils.getTrajectory(
                        script,
                        startingPose,
                        2); // Get second movement trajectory

                FollowTrajectory tag_1_movement2 = new FollowTrajectory(
                        container.m_robotDrive,
                        tag_1_traj2,
                        startingPose.getRotation(),
                        true); // Create a new movement command for the second movement

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
                         tag_1_movement2, // Do second Movement
                        new InstantCommand(
                                () -> container.m_robotDrive.stopModules())); // Ensure Robot Is Stopped;
            case TAG_2_RED:
            
            startingPose = container.m_robotDrive.getPose(); // Setup starting pose
            Trajectory tag_2_traj1 = AutonUtils.getTrajectory(
                    script,
                    startingPose,
                    1); // Get first movement trajectory
            FollowTrajectory tag_2_movement1 = new FollowTrajectory(
                    container.m_robotDrive,
                    tag_2_traj1,
                    startingPose.getRotation(),
                    false); // Create a new movement command for the first movement

            startingPose = new Pose2d(
                    Units.feetToMeters(19),
                    Units.feetToMeters(10),
                    startingPose.getRotation()); // Update starting pose for next movement

            Trajectory tag_2_traj2 = AutonUtils.getTrajectory(
                    script,
                    startingPose,
                    2); // Get second movement trajectory

            FollowTrajectory tag_2_movement2 = new FollowTrajectory(
                    container.m_robotDrive,
                    tag_2_traj2,
                    startingPose.getRotation(),
                    true); // Create a new movement command for the second movement

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

                    // tag_2_movement1, // Do First Movement
                    // tag_2_movement2, // Do second Movement
                    new InstantCommand(
                            () -> container.m_robotDrive.stopModules())); // Ensure Robot Is Stopped;
            case TAG_3_RED:
            
            startingPose = container.m_robotDrive.getPose(); // Setup starting pose
            Trajectory tag_3_traj1 = AutonUtils.getTrajectory(
                    script,
                    startingPose,
                    1); // Get first movement trajectory
            FollowTrajectory tag_3_movement1 = new FollowTrajectory(
                    container.m_robotDrive,
                    tag_3_traj1,
                    startingPose.getRotation(),
                    false); // Create a new movement command for the first movement

            startingPose = new Pose2d(
                    Units.feetToMeters(19),
                    Units.feetToMeters(10),
                    startingPose.getRotation()); // Update starting pose for next movement

            Trajectory tag_3_traj2 = AutonUtils.getTrajectory(
                    script,
                    startingPose,
                    2); // Get second movement trajectory

            FollowTrajectory tag_3_movement2 = new FollowTrajectory(
                    container.m_robotDrive,
                    tag_3_traj2,
                    startingPose.getRotation(),
                    true); // Create a new movement command for the second movement

            return new SequentialCommandGroup( // This runs the movements in order
                    new InstantCommand(
                            () -> container.m_robotDrive.resetOdometry(
                                    AutonUtils.getStartingPose(
                                        tag_3_traj1,
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

                    // tag_3_movement1, // Do First Movement
                    // tag_3_movement2, // Do second Movement
                    new InstantCommand(
                            () -> container.m_robotDrive.stopModules())); // Ensure Robot Is Stopped;
            default:
                return new SequentialCommandGroup();
        }
    }

    public static enum Script {
        SCORE_AND_CHARGE,
        TAG_8_BLUE,
        TAG_7_BLUE,
        TAG_6_BLUE,
        TAG_1_RED,
        TAG_2_RED,
        TAG_3_RED
    }
}
