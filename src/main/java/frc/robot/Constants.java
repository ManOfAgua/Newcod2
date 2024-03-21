package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Meters;

public class Constants {

        public static final class IDConstants {
                public static final int gyro = 17;
        }

        public static final class ControllerConstants { // for playstation 5 controller
                public static final int driver = 0,
                                operator = 1,
                                b_SQR = 1,
                                b_X = 2,
                                b_O = 3,
                                b_TRI = 4,
                                b_L1 = 5,
                                b_R1 = 6,
                                b_L2 = 7,
                                b_R2 = 8,
                                b_PIC = 9,
                                b_MEN = 10,
                                b_LJOY = 11,
                                b_RJOY = 12,
                                b_LOG = 13,
                                b_PAD = 14,
                                b_MIC = 15;
        }

        public static final class IntakeConstants {
                public static final int intakeID = 10;

                public static double intakeSpd = 0.6;
        }

        public static final class ShooterConstants {
                public static final int shooterTopID = 11,
                                shooterBtmID = 12;

                public static double shooterSpd = 0.8,
                                shooterSlwSpd = 0.2,
                                shooterKP = 0.1, shooterKI = 0.0, shooterKD = 0.0;
        }

        public static final class ArmConstants {
                public static final int leftarmID = 9, rightarmID = 8;

                public static double armSpd = 0.40,
                                armKP = 0.01, armKI = 0.001, armKD = 0.0001,

                                kArmGearRatio = 36.66, kCountsPerRev = 2048,
                                kArmScaleFactor = (360 / (243.316601563 / (kCountsPerRev * kArmGearRatio)));
        }

        public static final class AutonConstants {
                public static double drivekP = 0.5,
                                drivekI = 0.0,
                                drivekD = 0.1,

                                steerkP = 0.5,
                                steerkI = 0.0,
                                steerkD = 0.1;
        }

        public static class VisionConstants {
                /**
                 * Array of PhotonVision camera names. The values here match
                 * ROBOT_TO_CAMERA_TRANSFORMS for the camera's location.
                 */
                public static final String[] APRILTAG_CAMERA_NAME = { "PhotonCamera" };

                /**
                 * Physical location of the apriltag cameras on the robot, relative to the
                 * center of the robot.
                 * The values here math APRILTAG_CAMERA_NAME for the camera's name.
                 */
                public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = {
                                new Transform3d(
                                                new Translation3d(inchesToMeters(14), inchesToMeters(14),
                                                                inchesToMeters(7.75)),
                                                new Rotation3d(0, degreesToRadians(0), degreesToRadians(0))),
                                new Transform3d(
                                                new Translation3d(inchesToMeters(7.678), inchesToMeters(12.333),
                                                                inchesToMeters(10.619)),
                                                new Rotation3d(degreesToRadians(-0.25), degreesToRadians(-25),
                                                                degreesToRadians(0)))
                };

                public static final Measure<Distance> FIELD_LENGTH = Meters.of(16.54175);
                public static final Measure<Distance> FIELD_WIDTH = Meters.of(8.0137);

                /**
                 * Minimum target ambiguity. Targets with higher ambiguity will be discarded.
                 * Not appliable when multiple tags are
                 * in view in a single camera.
                 */
                public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        }

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(22.75 / 2.0, 22.75 / 2.0),
                        // Front right
                        new Translation2d(22.75 / 2.0, -22.75 / 2.0),
                        // Back left
                        new Translation2d(-22.75 / 2.0, 22.75 / 2.0),
                        // Back right
                        new Translation2d(-22.75 / 2.0, -22.75 / 2.0));
}
