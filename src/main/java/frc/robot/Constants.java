package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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
                                armKP = 0.02, armKI = 0.001, armKD = 0.003,

                                kArmGearRatio = 36.66, kCountsPerRev = 2048,
                                kArmScaleFactor = (360 / (243.316601563 / (kCountsPerRev * kArmGearRatio)));
        }

        public static final class AutonConstants {
                public static double drivekP = 0.5,
                                drivekI = 0.0,
                                drivekD = 0.1,
                                // 29/64 nick took
                                steerkP = 0.5,
                                steerkI = 0.0,
                                steerkD = 0.1;
        }

        public static final class VisionConstants {
                public static final String APRILTAG_CAMERA_NAME = "PhotonCamera";

                public static final AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFields.k2024Crescendo
                                .loadAprilTagLayoutField();

                public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

                /**
                 * Physical location of the apriltag camera on the robot, relative to the center
                 * of the robot.
                 */
                public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
                                new Translation3d(-0.06, 0.2, -0.2127),
                                new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(-3.0)));

                public static final double FIELD_LENGTH_METERS = 16.54175;
                public static final double FIELD_WIDTH_METERS = 8.0137;

                /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
                public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

                public static final Pose2d FLIPPING_POSE = new Pose2d(
                                new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
                                new Rotation2d(Math.PI));

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