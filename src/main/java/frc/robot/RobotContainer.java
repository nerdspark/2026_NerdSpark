// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.TuneTurretCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SimPoseSubsystem;
import frc.robot.subsystems.SimFuelSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.util.FuelSim;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final PoseEstimatorSubsystem poseEstimatorSubsystem;

    private final SendableChooser<Command> autoChooser;

    private final Turret turret;
    private final SimFuelSubsystem fuelSim;
    private final SimPoseSubsystem simPose;

    public RobotContainer() {
        poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        turret = new Turret(
            () -> drivetrain.getState().Pose, 
            () -> drivetrain.getState().Speeds,
            () -> DriverStation.getAlliance().orElse(Alliance.Red),
            () -> true,
            logger
        );
        turret.setDefaultCommand(new TuneTurretCommand(turret));

        fuelSim = RobotBase.isSimulation()
            ? new SimFuelSubsystem(
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds
            )
            : null;
        simPose = RobotBase.isSimulation()
            ? new SimPoseSubsystem(drivetrain)
            : null;

        configureDefaultCommands();
        // configureSysid();

        configureBindings();
    }

    private void configureBindings() {
        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.x().onTrue(new InstantCommand(() -> FuelSim.getInstance().spawnFuel(new Translation3d(poseEstimatorSubsystem.getCurrentPose().getX(), poseEstimatorSubsystem.getCurrentPose().getY(), Units.inchesToMeters(30)), launchVel(7.5, 80)))
.andThen(Commands.waitSeconds(0.25))
.repeatedly().until(() -> !joystick.x().getAsBoolean()));
    }

    private void configureDefaultCommands() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getRightY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getRightX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureSysid() {
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
            Units.inchesToMeters(25),
            Units.inchesToMeters(29),
            Units.inchesToMeters(25),
            () -> poseEstimatorSubsystem.getCurrentPose(),
            () -> drivetrain.getCurrentRobotChassisSpeeds());
    instance.registerIntake(
            -Units.inchesToMeters(26),
            Units.inchesToMeters(26),
            -Units.inchesToMeters(26),
            Units.inchesToMeters(26),
            () -> true);
    // instance.registerIntake(
    //         -Units.inchesToMeters(26),
    //         Units.inchesToMeters(26),
    //         Units.inchesToMeters(26),
    //         Units.inchesToMeters(26),
    //         () -> true);

    instance.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
}

private Translation3d launchVel(double vel, double angle) {
        // Pose3d robot = poseSupplier.get();
        Translation3d robotTranslation = new Translation3d(poseEstimatorSubsystem.getCurrentPose().getX(), poseEstimatorSubsystem.getCurrentPose().getY(), Units.inchesToMeters(30));
        Pose3d robot = new Pose3d(robotTranslation, new Rotation3d(poseEstimatorSubsystem.getCurrentPose().getRotation()));
        ChassisSpeeds fieldSpeeds = drivetrain.getCurrentRobotChassisSpeeds();

        double horizontalVel = Math.cos(Units.degreesToRadians(angle)) * vel;
        double verticalVel = Math.sin(Units.degreesToRadians(angle)) * vel;
        double xVel =
                horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
        double yVel =
                horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }
}
