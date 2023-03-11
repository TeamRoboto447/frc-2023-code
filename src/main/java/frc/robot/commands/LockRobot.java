// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class LockRobot extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final DriveSubsystem driveSubsystem;
  /** Creates a new HoldArmStill. */
  public LockRobot(ArmSubsystem aSubsystem, DriveSubsystem dSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = aSubsystem;
    this.driveSubsystem = dSubsystem;
    addRequirements(aSubsystem);
    addRequirements(dSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.armSubsystem.holdAll();
    this.driveSubsystem.setBrakeMode(true);
    this.driveSubsystem.setModuleStates(
      new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      }
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
