package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.autoBalance;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
    private DriveSubsystem drivetrain;
    private autoBalance autoBalance;

    public AutoBalance(DriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        autoBalance = new autoBalance(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(0, autoBalance.autoBalanceRoutine(), 0, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
    }
}
