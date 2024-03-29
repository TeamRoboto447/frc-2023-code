// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutonUtils;

public class FollowTrajectoryDeadReckoning extends SwerveControllerCommand {
  /** Creates a new FollowTrajectory. */
  private final DriveSubsystem driveSubsystem;
  private final Trajectory trajectory;
  private final boolean finalMovement;
  public FollowTrajectoryDeadReckoning(DriveSubsystem dSubsystem, Trajectory traj, Rotation2d rotation, boolean lastMove) {
    super(traj, dSubsystem::getRawOdometryPose, DriveConstants.kDriveKinematics, AutoConstants.xController, AutoConstants.yController, AutoConstants.thetaController, () -> rotation, dSubsystem::setModuleStates, dSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = dSubsystem;
    this.trajectory = traj;
    this.finalMovement = lastMove;
    addRequirements(dSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AutoConstants.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.driveSubsystem.updateEstimationFromVision();
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if(this.finalMovement) {
      this.driveSubsystem.setBrakeMode(true);
      this.driveSubsystem.stopModules();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
