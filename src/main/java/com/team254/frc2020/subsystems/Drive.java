package com.team254.frc2020.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team254.frc2020.Constants;
import com.team254.frc2020.Kinematics;
import com.team254.frc2020.RobotState;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.frc2020.planners.DriveMotionPlanner;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

public class Drive extends Subsystem {
    private static Drive mInstance;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    public static class PeriodicIO {
        // INPUTS
        public Rotation2d gyro_heading = Rotation2d.identity();
        public double timestamp = Timer.getFPGATimestamp(); // used for vel and acc

        // Driver controls
        public double forward;
        public double strafe;
        public double rotation;
        public boolean low_power;
        public boolean field_relative;
        public boolean use_heading_controller;

        // OUTPUTS
        public double[] wheel_speeds = new double[] {0, 0, 0, 0};
        public Rotation2d[] wheel_azimuths = new Rotation2d[]{Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity()};
        public Pose2d error = Pose2d.identity();
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }

    public enum ControlState {
        OPEN_LOOP,
        PATH_FOLLOWING
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mDriveControlState = ControlState.OPEN_LOOP;

    private DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner();

    private boolean mOverrideTrajectory = false;

    private PigeonIMU mPigeonIMU;
    private SwerveModule[] mModules = new SwerveModule[4];

    private Rotation2d mGyroOffset = Rotation2d.identity();

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private Drive() {
        mPigeonIMU = new PigeonIMU(Constants.kPigeonId);

        mModules[0] = new SwerveModule(Constants.kFrontLeftModuleConstants);
        mModules[1] = new SwerveModule(Constants.kFrontRightModuleConstants);
        mModules[2] = new SwerveModule(Constants.kBackRightModuleConstants);
        mModules[3] = new SwerveModule(Constants.kBackLeftModuleConstants);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != ControlState.OPEN_LOOP) {
            mDriveControlState = ControlState.OPEN_LOOP;
        }

        DriveSignal adjusted = Util.adjustDriveSignal(signal, getModuleAzimuths());
        mPeriodicIO.wheel_speeds = adjusted.getWheelSpeeds();
        mPeriodicIO.wheel_azimuths = adjusted.getWheelAzimuths();
    }

    public void setTeleopInputs(double forward, double strafe, double rotation, boolean low_power, boolean field_relative, boolean use_heading_controller) {
        if (mDriveControlState != ControlState.OPEN_LOOP) {
            mDriveControlState = ControlState.OPEN_LOOP;
        }
        mPeriodicIO.forward = forward;
        mPeriodicIO.strafe = strafe;
        mPeriodicIO.rotation = rotation;
        mPeriodicIO.low_power = low_power;
        mPeriodicIO.field_relative = field_relative;
        mPeriodicIO.use_heading_controller = use_heading_controller;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeonIMU.getFusedHeading()).rotateBy(mGyroOffset);

        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        for (int i = 0; i < mModules.length; i++) {
            if (mModules != null && mModules[i] != null) {
                mModules[i].setOpenLoop(mPeriodicIO.wheel_speeds[i], mPeriodicIO.wheel_azimuths[i]);
            }
        }
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    startLogging();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            setOpenLoop(SwerveDriveHelper.calculateDriveSignal(mPeriodicIO.forward,
                                    mPeriodicIO.strafe, mPeriodicIO.rotation, mPeriodicIO.low_power,
                                    mPeriodicIO.field_relative, mPeriodicIO.use_heading_controller));
                            break;
                        case PATH_FOLLOWING:
                            updatePathFollower();
                            break;
                        default:
                            System.out.println("Unexpected control state: " + mDriveControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
            }
        });
    }

    public SwerveModule[] getSwerveModules() {
        return mModules;
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());

        for (int i = 0; i < mModules.length; i++) {
            if (mModules != null && mModules[i] != null) {
                mModules[i].zeroSensors();
            }
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("FW", mPeriodicIO.forward);
        SmartDashboard.putNumber("STR", mPeriodicIO.strafe);
        SmartDashboard.putNumber("Rotation", mPeriodicIO.rotation);
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public void setOverrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    public Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeonIMU.getFusedHeading()).inverse());

        mPeriodicIO.gyro_heading = heading;
    }

    /**
     * @param trajectory Desired robot trajectory
     */
    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = ControlState.PATH_FOLLOWING;
        }
    }

    // TODO
    Translation2d lastDriveVector = Translation2d.identity();
    private void updatePathFollower() {
        if (mDriveControlState == ControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();
            double rotationCorrection = SwerveHeadingController.getInstance().update();

            Translation2d driveVector = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            mPeriodicIO.error = mMotionPlanner.getError();
            mPeriodicIO.path_setpoint = mMotionPlanner.getSetpoint();
            double rotationInput = Deadband.apply(rotationCorrection * driveVector.norm(), mMotionPlanner.getMaxRotationSpeed());

            if (!mOverrideTrajectory) {
                if (Util.epsilonEquals(driveVector.norm(), 0.0, Util.kEpsilon)) {
                    driveVector = lastDriveVector;
                    setDriveOutputs(Kinematics.inverseKinematics(driveVector.x(), driveVector.y(), rotationInput, true), 0.0);
                } else {
                    setDriveOutputs(Kinematics.inverseKinematics(driveVector.x(), driveVector.y(), rotationInput, true));
                }
            } else {
                DriveSignal signal = DriveSignal.BRAKE;
                setDriveOutputs(Util.adjustDriveSignal(signal, getModuleAzimuths()));
            }

            lastDriveVector = driveVector;
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized double[] getModuleVelocities() {
        double[] ret_val = new double[mModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = mModules[i].getLinearVelocity();
        }

        return ret_val;
    }

    public synchronized Rotation2d[] getModuleAzimuths() {
        Rotation2d[] ret_val = new Rotation2d[mModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = mModules[i].getAngle();
        }

        return ret_val;
    }

    /**
     * Configures each module to match its assigned vector, but puts the drive motors into closed-loop velocity mode
     */
    public void setDriveOutputs(DriveSignal signal) {
        double[] velocities = new double[mModules.length];
        Rotation2d[] azimuths = new Rotation2d[mModules.length];
        for (int i = 0; i < signal.getWheelAzimuths().length; i++) {
            velocities[i] = signal.getWheelSpeeds()[i];
            azimuths[i] = signal.getWheelAzimuths()[i];
        }
        DriveSignal adjusted = Util.adjustDriveSignal(new DriveSignal(velocities, azimuths), getModuleAzimuths());
        mPeriodicIO.wheel_speeds = adjusted.getWheelSpeeds();
        mPeriodicIO.wheel_azimuths = adjusted.getWheelAzimuths();
    }

    public void setDriveOutputs(DriveSignal signal, double velocityOverride) {
        double[] velocities = new double[mModules.length];
        Rotation2d[] azimuths = new Rotation2d[mModules.length];
        for (int i = 0; i < signal.getWheelAzimuths().length; i++) {
            velocities[i] = velocityOverride;
            azimuths[i] = signal.getWheelAzimuths()[i];
        }
        DriveSignal adjusted = Util.adjustDriveSignal(new DriveSignal(velocities, azimuths), getModuleAzimuths());
        mPeriodicIO.wheel_speeds = adjusted.getWheelSpeeds();
        mPeriodicIO.wheel_azimuths = adjusted.getWheelAzimuths();
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != ControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
}