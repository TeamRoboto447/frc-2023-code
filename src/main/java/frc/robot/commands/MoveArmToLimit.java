// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToLimit extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final double targetIntakeSpeed;
  private final Limit verticalLimit;
  private final Limit horizontalLimit;
  private boolean verticalDone = false;
  private boolean horizontalDone = false;

  /** Creates a new ExtendAndLift. */
  public MoveArmToLimit(ArmSubsystem aSubsystem, Limit vLimit, Limit hLimit, double tIntakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = aSubsystem;
    this.targetIntakeSpeed = tIntakeSpeed;

    this.verticalLimit = vLimit;
    this.horizontalLimit = hLimit;

    addRequirements(aSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.armSubsystem.setMaxArmSpeeds(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.verticalLimit == Limit.NO_CHANGE) {
      this.verticalDone = true;
      stopVert();
    }
    else if (this.verticalLimit == Limit.TOP_VERTICAL)
      this.verticalDone = runVert(ArmConstants.vertBangBangSpeed);
    else if (this.verticalLimit == Limit.BOTTOM_VERTICAL)
    this.verticalDone = runVert(-ArmConstants.vertBangBangSpeed);

    if (this.horizontalLimit == Limit.NO_CHANGE) {
      this.horizontalDone = true;
      stopHoriz();
    }
    else if (this.horizontalLimit == Limit.FAR_HORIZONTAL)
      this.horizontalDone = runHoriz(ArmConstants.horizBangBangSpeed);
    else if(this.horizontalLimit == Limit.CLOSE_HORIZONTAL)
      this.horizontalDone = runHoriz(-ArmConstants.horizBangBangSpeed);

    this.runIntake(targetIntakeSpeed);
  }

  private boolean runVert(double speed) {
    this.armSubsystem.teleopMoveVertical(speed);
    this.armSubsystem.setVertTarget(this.armSubsystem.getVertEncoder());
    return speed > 0 ? this.armSubsystem.atVerticalHighLimit() : this.armSubsystem.atVerticalLowLimit();
  }

  private boolean runHoriz(double speed) {
    this.armSubsystem.rawMoveHorizontal(speed);
    this.armSubsystem.setDistTarget(this.armSubsystem.getHorizontalEncoder());
    return speed > 0 ? this.armSubsystem.atHorizontalHighLimit() : this.armSubsystem.atHorizontalLowLimit();
  }

  private void stopVert() {
 //   this.armSubsystem.goToVertical(this.armSubsystem.getVertEncoder());
    this.armSubsystem.rawMoveVertical(0);
  }

  private void stopHoriz() {
//this.armSubsystem.goToHorizontal(this.armSubsystem.getHorizontalEncoder());
    this.armSubsystem.rawMoveHorizontal(0);
  }

  private void runIntake(double tRotSpeed) {
    this.armSubsystem.rawIntakeGrabber(tRotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.setMaxArmSpeeds(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.verticalDone && this.horizontalDone;
  }

  public static enum Limit {
    TOP_VERTICAL,
    BOTTOM_VERTICAL,
    FAR_HORIZONTAL,
    CLOSE_HORIZONTAL,
    NO_CHANGE
  }
}
