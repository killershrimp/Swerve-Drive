package com.team254.lib.util;

import com.team254.lib.geometry.Rotation2d;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {}

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double bound0To2PIRadians(double radians) {
        return Math.toRadians(bound0To360Degrees(Math.toDegrees(radians)));
    }

    public static double bound0To360Degrees(double degrees) {
        degrees %= 360;
        if (degrees < 0) {
            degrees += 360;
        }

        return degrees;
    }

    /**
     * Optimizes azimuth setpoints and drive velocity setpoints so no needed azimuth rotation is > pi/2 radians
     * and adjusts drive velocity direction accordingly
     *
     * @param signal Raw DriveSignal
     */
    public static DriveSignal adjustDriveSignal(DriveSignal signal, Rotation2d[] currentAzis) {
        Rotation2d[] aziSetpoints = signal.getWheelAzimuths();
        double[] velSetpoints = signal.getWheelSpeeds();

        for (int i = 0; i < 4; i++) {
            double boundedAziSetpoint = Util.bound0To2PIRadians(aziSetpoints[i].getRadians());
            double boundedAziCurrent = Util.bound0To2PIRadians(currentAzis[i].getRadians());
            double error = Util.bound0To2PIRadians(boundedAziSetpoint - boundedAziCurrent);
            if (error > Math.PI) {
                error -= Math.PI;
            } else if (error < -Math.PI) {
                error += Math.PI;
            }

            if (Math.abs(error) > Math.PI / 2) {
                velSetpoints[i] *= -1;

                aziSetpoints[i] = Rotation2d.fromRadians(oppositeAngle(aziSetpoints[i]));
            }
        }
        return new DriveSignal(velSetpoints, aziSetpoints);
    }

    private static double oppositeAngle(Rotation2d angle) {
        double angleM = Util.bound0To2PIRadians(angle.getRadians());
        angleM += Math.PI;
        return angleM % (2 * Math.PI);
    }
}
