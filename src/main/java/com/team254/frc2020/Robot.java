package com.team254.frc2020;

import com.team254.frc2020.auto.AutoModeExecutor;
import com.team254.frc2020.auto.modes.AutoModeBase;
import com.team254.frc2020.controlboard.ControlBoard;
import com.team254.frc2020.controlboard.IControlBoard;
import com.team254.frc2020.loops.Looper;
import com.team254.frc2020.subsystems.*;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.*;
import com.team254.lib.wpilib.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class Robot extends TimedRobot {
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final Drive mDrive = Drive.getInstance();

    private final SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();

    private IControlBoard mControlBoard = ControlBoard.getInstance();

    private TimeDelayedBoolean mHangModeEnablePressed = new TimeDelayedBoolean();
    private TimeDelayedBoolean mHangModeLowEnablePressed = new TimeDelayedBoolean();
    private boolean mInHangMode;
    private boolean mIntakeButtonPressed = false;
    private boolean mHangModeReleased = true;

    private MultiTrigger mDiskIntakeTrigger = new MultiTrigger(.4);
    private MultiTrigger mBallIntakeTrigger = new MultiTrigger(.4);

    private boolean mHasBeenEnabled = false;
    private DigitalInput mResetButton = new DigitalInput(Constants.kResetButtonChannel);

    private double mLastShootPressedTime = -1.0;
    private double mOffsetOverride = -1.0;

    private LatchedBoolean mShootPressed = new LatchedBoolean();
    private boolean mStickyShoot;

    private LatchedBoolean mThrustReleased = new LatchedBoolean();
    private LatchedBoolean mThrustPressed = new LatchedBoolean();
    private double mLastThrustPressedTime = -1.0;
    private double mLastThrustShotTime = Double.NaN;

    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;
    private boolean mDriveByCameraInAuto = false;

    Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(new Subsystem[] {mRobotStateEstimator, mDrive},
        mDrive.getSwerveModules());

            System.out.println("ZEROING ROBOT");

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());

            mAutoModeSelector.updateModeCreator();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDrive.zeroSensors();

            mDisabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            mHasBeenEnabled = true;

            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.zeroSensors();

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            mHasBeenEnabled = true;

            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            mEnabledLooper.start();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            // Force true on first iteration of teleop periodic
            shouldChangeAzimuthSetpoint.update(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    LatchedBoolean shouldChangeAzimuthSetpoint = new LatchedBoolean();

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        try {
            if (!mResetButton.get() && !mHasBeenEnabled) {
                System.out.println("Zeroing Robot!!!!");
            }

            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            mDrive.setHeading(Rotation2d.identity());
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            mDriveByCameraInAuto = mAutoModeSelector.isDriveByCamera();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        try {
            if (mDriveByCameraInAuto) {
                manualControl();
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    TimeDelayedBoolean mShouldMaintainAzimuth = new TimeDelayedBoolean();

    @Override
    public void teleopPeriodic() {
        try {
            manualControl();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}

    public void manualControl() {
        // drive
        boolean maintainAzimuth = mShouldMaintainAzimuth.update(mControlBoard.getRotation() == 0, 0.2);
        boolean changeAzimuthSetpoint = shouldChangeAzimuthSetpoint.update(maintainAzimuth);

        if (mControlBoard.getDPad() != -1) {
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.SNAP);
            double heading_goal = mControlBoard.getDPad();
            SmartDashboard.putNumber("Heading Goal", heading_goal);
            mSwerveHeadingController.setGoal(heading_goal);
        } else {
            if (!maintainAzimuth) {
                mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            } else if ((mSwerveHeadingController.getHeadingControllerState() == SwerveHeadingController.HeadingControllerState.SNAP
                    && mSwerveHeadingController.isAtGoal()) || changeAzimuthSetpoint) {
                mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
                mSwerveHeadingController.setGoal(mDrive.getHeading().getDegrees());
            }
        }

        if (mSwerveHeadingController.getHeadingControllerState() != SwerveHeadingController.HeadingControllerState.OFF) {
            mDrive.setTeleopInputs(mControlBoard.getThrottle(), mControlBoard.getStrafe(), mSwerveHeadingController.update(),
                    mControlBoard.getDriveLowPower(), mControlBoard.getFieldRelative(), true);
        } else {
            mDrive.setTeleopInputs(mControlBoard.getThrottle(), mControlBoard.getStrafe(), mControlBoard.getRotation(),
                    mControlBoard.getDriveLowPower(), mControlBoard.getFieldRelative(), false);
        }

    }
}