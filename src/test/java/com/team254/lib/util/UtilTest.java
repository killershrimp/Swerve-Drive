package com.team254.lib.util;


import com.team254.lib.geometry.Rotation2d;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

/**
 * Class that tests the system test
 */
@RunWith(JUnit4.class)
public class UtilTest {
    @Test
    public void testBounding() {
        Assert.assertEquals(0, Util.bound0To2PIRadians(4 * Math.PI), Util.kEpsilon);
        Assert.assertEquals(Math.PI / 2, Util.bound0To2PIRadians(5 * Math.PI / 2), Util.kEpsilon);
        Assert.assertEquals(7 * Math.PI / 8, Util.bound0To2PIRadians(-25 * Math.PI / 8), Util.kEpsilon);
        Assert.assertEquals(3 * Math.PI / 2, Util.bound0To2PIRadians(-5 * Math.PI / 2), Util.kEpsilon);
    }

    public void testAdjustDriveSignal(DriveSignal desired, Rotation2d[] currentAzis, DriveSignal expected) {
        DriveSignal result = Util.adjustDriveSignal(desired, currentAzis);

        Assert.assertArrayEquals(result.getWheelSpeeds(), expected.getWheelSpeeds(), Util.kEpsilon);
        for (int i = 0; i < 4; i++) {
            Assert.assertEquals(result.getWheelAzimuths()[i].getRadians(), expected.getWheelAzimuths()[i].getRadians(), Util.kEpsilon);
        }
    }

    @Test
    public void test() {
        Rotation2d[] current1 = {
                Rotation2d.fromRadians(Math.PI),
                Rotation2d.fromRadians(Math.PI),
                Rotation2d.fromRadians(3 * Math.PI / 2),
                Rotation2d.fromRadians(5 * Math.PI / 7)
        };

        DriveSignal desired1 = new DriveSignal(
                new double[]{1, -1, -1, 0.57},
                new Rotation2d[] {
                        Rotation2d.fromRadians(Math.PI / 2),
                        Rotation2d.fromRadians(0),
                        Rotation2d.fromRadians(Math.PI / 2),
                        Rotation2d.fromRadians(11 * Math.PI / 7)
                }
        );

        DriveSignal expected1 = new DriveSignal(
                new double[]{1, 1, 1, -0.57},
                new Rotation2d[]{
                        Rotation2d.fromRadians(Math.PI / 2),
                        Rotation2d.fromRadians(Math.PI),
                        Rotation2d.fromRadians(3 * Math.PI / 2),
                        Rotation2d.fromRadians(4 * Math.PI / 7)
                }
        );
        testAdjustDriveSignal(desired1, current1, expected1);
    }
}