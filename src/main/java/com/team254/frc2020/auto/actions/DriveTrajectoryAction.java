package com.team254.frc2020.auto.actions;

import com.team254.frc2020.RobotState;
import com.team254.frc2020.subsystems.Drive;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrajectoryAction implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private static final RobotState mRobotState = RobotState.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final Rotation2d mAbsGoalHeading;
    private final boolean mResetPose;

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Rotation2d absGoalHeading) {
        this(trajectory, absGoalHeading, false);
    }


    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Rotation2d absGoalHeading, boolean resetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mAbsGoalHeading = absGoalHeading;
        mResetPose = resetPose;
    }

    @Override
    public void start() {
        System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose) {
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
        }
        mDrive.setTrajectory(mTrajectory, mAbsGoalHeading);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            System.out.println("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void done() {}
}

