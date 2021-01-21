package com.team254.frc2020.controlboard;

import com.team254.frc2020.Constants;
import com.team254.lib.util.Deadband;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private final double kDeadband = 0.15;
    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadButtonControlBoard() {
        mController = new XboxController(Constants.kButtonGamepadPort);
    }
}