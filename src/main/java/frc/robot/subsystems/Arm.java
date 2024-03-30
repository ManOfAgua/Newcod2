// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  TalonFX leftArm = new TalonFX(ArmConstants.leftarmID);
  TalonFX rightArm = new TalonFX(ArmConstants.rightarmID);
  PhotonCamera photonCamera = new PhotonCamera("PhotonCamera");
  Follower follower = new Follower(ArmConstants.leftarmID, true);
  CommandSwerveDrivetrain m_driveTrain;
  // private final double camera_Height = Units.inchesToMeters(10);
  // private final double target_Height = Units.inchesToMeters(56.125); // Distance from floor to the middle of april tag
  // private AprilTagFieldLayout aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  //   private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");


  // private final double vertoffSet = Units.inchesToMeters(24.3125); // 80.4375 height of middle of speaker opening..
                                                                   // 80.4375-56.125=24.3125

  public Arm(CommandSwerveDrivetrain driveTrain) {
    brakeMode();
    currentlimit();
    this.m_driveTrain = driveTrain;
    rightArm.setControl(follower);
    // this.aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  public void move(double speed) {
    leftArm.set(speed);
  }

  public void currentlimit() {
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    leftArm.getConfigurator().refresh(currentLimits);
    rightArm.getConfigurator().refresh(currentLimits);

    currentLimits.SupplyCurrentLimit = 70;
    currentLimits.SupplyCurrentThreshold = 80;
    currentLimits.SupplyTimeThreshold = 0.1;
    currentLimits.SupplyCurrentLimitEnable = true;
    leftArm.getConfigurator().apply(currentLimits);
    rightArm.getConfigurator().apply(currentLimits);
  }

  public double armTickToDegrees() {
    double motorRotations = leftArm.getPosition().getValueAsDouble()
        / (ArmConstants.kCountsPerRev * ArmConstants.kArmGearRatio);
    double armTicksPerDegree = motorRotations * ArmConstants.kArmScaleFactor;
    double offset = 90;
    return -armTicksPerDegree + offset;
  }

  private void resetEncoders() {
    leftArm.setPosition(0);
    rightArm.setPosition(0);
  }

  public void brakeMode() {
    leftArm.setNeutralMode(NeutralModeValue.Brake);
    rightArm.setNeutralMode(NeutralModeValue.Brake);
  }

  // public double calculateAngle() {
  //   var result = photonCamera.getLatestResult();
  //   PhotonTrackedTarget target = result.getBestTarget();

  //   if (target == null) {
  //     return armTickToDegrees();
  //   }

  //   int targetID = target.getFiducialId();
  //   if (targetID == 7 || targetID == 4) {
  //     double horizontal = PhotonUtils.getDistanceToPose(m_driveTrain.getState().Pose,
  //         aprilTagLayout.getTagPose(targetID).get().toPose2d());

  //     double angle = Math.atan(((target_Height - camera_Height) + (vertoffSet)) / (horizontal));
  //     return Units.radiansToDegrees(angle);
  //   }
  //   return armTickToDegrees();
  // }


  // private SysIdRoutine m_armSysIdRoutine = new SysIdRoutine(
  // new SysIdRoutine.Config(null, null, null,
  // ModifiedSignalLogger.logState()),
  // new SysIdRoutine.Mechanism(
  // (Measure<Voltage> volts) -> leftArm.setVoltage(volts.in(volts2)),
  // null,
  // this));

  // public Command runArmQuasiTest(SysIdRoutine.Direction direction) {
  // return m_armSysIdRoutine.quasistatic(direction);
  // }

  // public Command runArmDynamTest(SysIdRoutine.Direction direction) {
  // return m_armSysIdRoutine.dynamic(direction);
  // }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Arm Angle", armTickToDegrees());
    // SmartDashboard.putNumber("Calculated Arm Angle", calculateAngle());

    // SmartDashboard.putNumber("Photon Angle", calculateAngle());

    // Test 1: -121.3916015625
    // Test 2: -122.13525390625
    // Test 3: -122.5244140625
    // Test 4: -119.46533203125
    // Test 5: -122.77490234375
    // Avg: -121.65830078125 * 2

    //////////////////////////// PHOTON /////////////////////

    // SmartDashboard.putNumber("ID Detected",
    // photonCamera.getLatestResult().getBestTarget().getFiducialId());
    // SmartDashboard.putNumber("Yaw April Tag",
    // photonCamera.getLatestResult().getBestTarget().getYaw());
    // SmartDashboard.putNumber("Calculating Camera Angle", calculateAngle());
  }
}