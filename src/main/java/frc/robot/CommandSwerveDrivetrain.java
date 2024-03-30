package frc.robot;

import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;

import java.util.function.Supplier;

// import static edu.wpi.first.units.Units.*;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.Util.ModifiedSignalLogger;
// import frc.robot.Util.SwerveVoltageRequest;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.Voltage;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    Pigeon2 gyro = new Pigeon2(IDConstants.gyro);

    // private final Thread photonThread = new Thread(new PhotonRunnable(APRILTAG_CAMERA_NAME, ROBOT_TO_CAMERA_TRANSFORMS,
    //         this::addVisionMeasurement, () -> getState().Pose));

    private PIDConstants drivePID = new PIDConstants(AutonConstants.drivekP, AutonConstants.drivekI,
            AutonConstants.drivekD);
    private PIDConstants steerPID = new PIDConstants(AutonConstants.steerkP, AutonConstants.steerkI,
            AutonConstants.steerkD);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        configurePathPlanner();
        driveLimit();
        azimuthLimit();
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyroYaw();
        // photonThread.setName("PhotonVision");
        // photonThread.setDaemon(true);
        // photonThread.start();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        configurePathPlanner();
        driveLimit();
        azimuthLimit();
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyroYaw();
        // photonThread.setName("PhotonVision");
        // photonThread.setDaemon(true);
        // photonThread.start();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(
                        drivePID,
                        steerPID,
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                        () -> DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false),
                        this); // Subsystem for requirements
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void driveLimit() {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        for (var swerveModule : Modules) {
            TalonFX driveMotor = swerveModule.getDriveMotor();
            driveMotor.getConfigurator().refresh(currentLimits);

            currentLimits.SupplyCurrentLimit = 40;
            currentLimits.SupplyCurrentThreshold = 55;
            currentLimits.SupplyTimeThreshold = 0.1;
            currentLimits.SupplyCurrentLimitEnable = true;
            driveMotor.getConfigurator().apply(currentLimits);
        }
    }

    public void azimuthLimit() {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        for (var swerveModule : Modules) {
            TalonFX driveMotor = swerveModule.getSteerMotor();
            driveMotor.getConfigurator().refresh(currentLimits);

            currentLimits.SupplyCurrentLimit = 20;
            currentLimits.SupplyCurrentThreshold = 25;
            currentLimits.SupplyTimeThreshold = 0.1;
            currentLimits.SupplyCurrentLimitEnable = true;
            driveMotor.getConfigurator().apply(currentLimits);
        }
    }

    public double getGyroYaw() {
        return gyro.getYaw().getValueAsDouble();
    }

    public void zeroGyroYaw() {
        gyro.setYaw(0);
    }

    // private SwerveVoltageRequest driveVoltageRequest = new
    // SwerveVoltageRequest(true);

    // private SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(null, null, null,
    // ModifiedSignalLogger.logState()),
    // new SysIdRoutine.Mechanism(
    // (Measure<Voltage> volts) ->
    // setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
    // null,
    // this));
    // private SwerveVoltageRequest steerVoltageRequest = new
    // SwerveVoltageRequest(false);

    // private SysIdRoutine m_steerSysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
    // new SysIdRoutine.Mechanism(
    // (Measure<Voltage> volts) ->
    // setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
    // null,
    // this));

    // private SysIdRoutine m_slipSysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null,
    // ModifiedSignalLogger.logState()),
    // new SysIdRoutine.Mechanism(
    // (Measure<Voltage> volts) ->
    // setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
    // null,
    // this));

    // public Command runDriveQuasiTest(Direction direction) {
    // return m_driveSysIdRoutine.quasistatic(direction);
    // }

    // public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    // return m_driveSysIdRoutine.dynamic(direction);
    // }

    // public Command runSteerQuasiTest(Direction direction) {
    // return m_steerSysIdRoutine.quasistatic(direction);
    // }

    // public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
    // return m_steerSysIdRoutine.dynamic(direction);
    // }

    // public Command runDriveSlipTest() {
    // return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    // }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Gyro Angle", getGyroYaw());
        SmartDashboard.putNumber("X Pose", this.getState().Pose.getX());
        SmartDashboard.putNumber("Y Pose", this.getState().Pose.getY());
        SmartDashboard.putNumber("Rotation Pose", this.getState().Pose.getRotation().getDegrees());

        for (int i = 0; i < Modules.length; i++) {
            SmartDashboard.putNumber("CANCODER ANGLES: " + i,
                    Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
        }
    }
}
