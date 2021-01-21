package com.team254.frc2020;

import com.team254.frc2020.auto.modes.AutoModeBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        // TODO add
    }

    enum DesiredMode {
        DRIVE_BY_CAMERA
        // TODO add
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        // TODO add default start options + other options

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Drive By Camera", DesiredMode.DRIVE_BY_CAMERA);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition staringPosition = mStartPositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || staringPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + staringPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, staringPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = staringPosition;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        switch (mode) {
            // TODO add modes
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DRIVE_BY_CAMERA;
    }
}