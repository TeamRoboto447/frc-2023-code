package frc.robot.utils;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.Constants.TeamSpecificConstants;
import frc.robot.subsystems.DriveSubsystem;

public class autoBalance {
    private BuiltInAccelerometer mRioAccel;
    private DriveSubsystem drivetrain;
    private int state;
    private int debounceCount;
    private double robotSpeedFine;
    private double robotSpeedSlow;
    private double robotSpeedFast;
    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;
    private double singleTapTime;
    private double scoringBackUpTime;
    private double doubleTapTime;

    public autoBalance(DriveSubsystem drivetrain) {
        mRioAccel = new BuiltInAccelerometer();
        this.state = 0;
        debounceCount = 0;
        this.drivetrain = drivetrain;

        /**********
         * CONFIG *
         **********/
        // Speed the robot drived while scoring/approaching station, default = 0.4
        robotSpeedFast = TeamSpecificConstants.is447Robot ? 5.5 : -7;

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        // default = 0.2
        robotSpeedSlow = TeamSpecificConstants.is447Robot ? 4.5 : -5.5;

        // Fine back and forth adjustment
        robotSpeedFine = TeamSpecificConstants.is447Robot ? 2.25 : -2.25;

        // Angle where the robot knows it is on the charge station, default = 13.0
        onChargeStationDegree = 13.0;

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence as well as for auto balancing,
        // default = 6.0
        levelDegree = TeamSpecificConstants.is447Robot ? 5 : 5.5;

        // Amount of time a sensor condition needs to be met before changing this.states in
        // seconds
        // Reduces the impact of sensor noice, but too high can make the auto run
        // slower, default = 0.2
        debounceTime = TeamSpecificConstants.is447Robot ? 0.2 : 0.2;

        // Amount of time to drive towards to scoring target when trying to bump the
        // game piece off
        // Time it takes to go from starting position to hit the scoring target
        singleTapTime = 0.4;

        // Amount of time to drive away from knocked over gamepiece before the second
        // tap
        scoringBackUpTime = 0.2;

        // Amount of time to drive forward to secure the scoring of the gamepiece
        doubleTapTime = 0.3;

    }

    // public double getPitch() {
    //     return Math.atan2((-mRioAccel.getX()),
    //             Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
    // }

    // public double getRoll() {
    //     return Math.atan2(mRioAccel.getY(), mRioAccel.getZ()) * 57.3;
    // }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    public double getTilt() {
        return drivetrain.getRoll();
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0, which left and right motors should be set
    // to.
    public double autoBalanceRoutine() {
        switch (this.state) {
            // drive forwards to approach station, exit when tilt is detected
            case 0:
                if (getTilt() > onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    this.state = 1;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 1:
                if (getTilt() < levelDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    this.state = 2;
                    debounceCount = 0;
                    return 0;
                }
                return robotSpeedSlow;
            // on charge station, stop motors and wait for end of auto
            case 2:
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    this.state = 3;
                    debounceCount = 0;
                    return 0;
                }
                if (getTilt() >= levelDegree) {
                    return robotSpeedFine;
                } else if (getTilt() <= -levelDegree) {
                    return -robotSpeedFine;
                }
            case 3:
                return 0;
        }
        return 0;
    }

    // Same as auto balance above, but starts auto period by scoring
    // a game piece on the back bumper of the robot
    public double scoreAndBalance() {
        switch (this.state) {
            // drive back, then forwards, then back again to knock off and score game piece
            case 0:
                debounceCount++;
                if (debounceCount < secondsToTicks(singleTapTime)) {
                    return -robotSpeedFast;
                } else if (debounceCount < secondsToTicks(singleTapTime + scoringBackUpTime)) {
                    return robotSpeedFast;
                } else if (debounceCount < secondsToTicks(singleTapTime + scoringBackUpTime + doubleTapTime)) {
                    return -robotSpeedFast;
                } else {
                    debounceCount = 0;
                    this.state = 1;
                    return 0;
                }
                // drive forwards until on charge station
            case 1:
                if (getTilt() > onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    this.state = 2;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 2:
                if (getTilt() < levelDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    this.state = 3;
                    debounceCount = 0;
                    return 0;
                }
                return robotSpeedSlow;
            // on charge station, ensure robot is flat, then end auto
            case 3:
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    this.state = 4;
                    debounceCount = 0;
                    return 0;
                }
                if (getTilt() >= levelDegree) {
                    return robotSpeedSlow / 2;
                } else if (getTilt() <= -levelDegree) {
                    return -robotSpeedSlow / 2;
                }
            case 4:
                return 0;
        }
        return 0;
    }
}
