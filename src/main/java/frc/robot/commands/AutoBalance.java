package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.autoBalance;

public class AutoBalance extends CommandBase {
    private final DriveSubsystem drivetrain;
    private final ArmSubsystem arm;
    private autoBalance autoBalance;
    private RollingAverage movement;

    public AutoBalance(DriveSubsystem drivetrain, ArmSubsystem arm) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.movement = new RollingAverage(5);
        autoBalance = new autoBalance(drivetrain);
    }

    @Override
    public void execute() {
        double requestedMovement = autoBalance.autoBalanceRoutine();
        movement.add(requestedMovement);
        drivetrain.drive(0, movement.getAverage(), 0, true);
        if(movement.getAverage() == 0) drivetrain.hold();
        else drivetrain.allowMovement();
        arm.goToVertical(130);
        arm.goToHorizontal(0);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
        drivetrain.allowMovement();
    }
}
