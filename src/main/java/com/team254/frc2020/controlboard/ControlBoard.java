package com.team254.frc2020.controlboard;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mDriveControlBoard = GamepadDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return mDriveControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return mDriveControlBoard.getRotation();
    }

    @Override
    public boolean getDriveLowPower() {
        return mDriveControlBoard.getDriveLowPower();
    }

    @Override
    public boolean getFieldRelative() {
        return mDriveControlBoard.getFieldRelative();
    }

    @Override
    public double getDPad() {
        return mDriveControlBoard.getDPad();
    }
}
