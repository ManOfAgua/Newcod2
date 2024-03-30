// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.ArmPIDFastCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmCommand;
// import frc.robot.commands.PhotonCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final PS5Controller driver = new PS5Controller(0);
  private final PS5Controller operator = new PS5Controller(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  /* Driver Buttons */
  private final JoystickButton pointButton = new JoystickButton(driver, ControllerConstants.b_X);
  private final JoystickButton brakeButton = new JoystickButton(driver, ControllerConstants.b_L2);
  private final JoystickButton reseedButton = new JoystickButton(driver, ControllerConstants.b_L1);
  private final JoystickButton revintakeButton = new JoystickButton(driver, ControllerConstants.b_R1);
  private final JoystickButton intakeButton = new JoystickButton(driver, ControllerConstants.b_R2);

  /* Operator Buttons */
  private final JoystickButton armFwdButton = new JoystickButton(operator, ControllerConstants.b_L2);
  private final JoystickButton armBckButton = new JoystickButton(operator, ControllerConstants.b_R2);
  private final JoystickButton armspeakerCloseButton = new JoystickButton(operator, ControllerConstants.b_TRI);
  private final JoystickButton shootButton = new JoystickButton(operator, ControllerConstants.b_O);
  private final JoystickButton shootSlowButton = new JoystickButton(operator, ControllerConstants.b_SQR);

  private final POVButton armPodiumButton = new POVButton(operator, 0);

  /* Subsystems */
  private final Arm armSub = new Arm(drivetrain);
  private final Shooter shooterSub = new Shooter();
  private final Intake intakeSub = new Intake();

  private final SendableChooser<Command> autoChooser;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                         // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
    pointButton.whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
    reseedButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    /* Driver Commands */
    intakeButton.whileTrue(new IntakeCommand(IntakeConstants.intakeSpd, intakeSub));
    revintakeButton.whileTrue(new IntakeCommand(-IntakeConstants.intakeSpd, intakeSub));
    revintakeButton.whileTrue(new ShooterCommand(-0.2, shooterSub));

    /* Operator Commands */
    shootButton.whileTrue(new ShooterCommand(ShooterConstants.shooterSpd, shooterSub));
    shootSlowButton.whileTrue(new ShooterCommand(ShooterConstants.shooterSlwSpd, shooterSub));

    armFwdButton.whileTrue(new ManualArmCommand(ArmConstants.armSpd, armSub));
    armBckButton.whileTrue(new ManualArmCommand(-ArmConstants.armSpd, armSub));

    armspeakerCloseButton.onTrue(new ArmPIDCommand(20, armSub));
    armPodiumButton.onTrue(new ArmPIDCommand(38, armSub));

    // photonCommandButton.onTrue(new PhotonCommand(armSub));

    /* Sysid Commands */
    // o_TRI.and(o_0).whileTrue(armSub.runArmQuasiTest(Direction.kForward));
    // o_TRI.and(o_180).whileTrue(armSub.runArmQuasiTest(Direction.kReverse));

    // o_SRQ.and(o_0).whileTrue(armSub.runArmDynamTest(Direction.kForward));
    // o_SRQ.and(o_180).whileTrue(armSub.runArmDynamTest(Direction.kReverse));

    // dr_sqr.and(dr_0).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
    // dr_sqr.and(dr_180).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));

    // dr_tri.and(dr_0).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
    // dr_tri.and(dr_180).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
    //TODO: Experiment with adding velocity to goal end state.
    //TODO: Reduce how far note intakes, only needs to intake a little bit.
    //TODO: Change path for fourth note so it is more direct intake.
    /*Note:
     * Event markets run while robot follows path, does not pause for event markers. 
     */
  public RobotContainer() {
    NamedCommands.registerCommand("Lower Arm", new ArmPIDFastCommand(20, armSub));
    NamedCommands.registerCommand("Raise Arm", new ArmPIDFastCommand(20, armSub));
    NamedCommands.registerCommand("FourthNoteRaise", new ArmPIDFastCommand(35, armSub));

    NamedCommands.registerCommand("Arm Ground", new ArmPIDCommand(-15.5, armSub));

    NamedCommands.registerCommand("Shoot", shooterSub.shootAuto(0.8).withTimeout(0.85)); //1.2

    NamedCommands.registerCommand("IntakeShoot", intakeSub.intakeAuto(0.5).withTimeout(0.85)); //1.1
    // Mid
    NamedCommands.registerCommand("PickupMid", intakeSub.intakeAuto(0.2).withTimeout(1.8));
    NamedCommands.registerCommand("PickupSource", intakeSub.intakeAuto(0.4).withTimeout(1.9));
    NamedCommands.registerCommand("PickupAmp", intakeSub.intakeAuto(0.4).withTimeout(1.7));
    // Amp
    NamedCommands.registerCommand("Note1", intakeSub.intakeAuto(0.4).withTimeout(1.95));
    NamedCommands.registerCommand("Note2", intakeSub.intakeAuto(0.4).withTimeout(3.83));
    //Source

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
    FollowPathCommand.warmupCommand().schedule();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());


    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivetrain::seedFieldRelative));

    RobotModeTriggers.teleop().onTrue(Commands.runOnce(drivetrain::seedFieldRelative));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(drivetrain::zeroGyroYaw));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
