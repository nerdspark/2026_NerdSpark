// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AimChassisCommand;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.IndexerCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.PoseEstimatorSubsystem;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // public final PoseEstimatorSubsystem poseEstimatorSubsystem;

    private final SendableChooser<Command> autoChooser;

    private final Indexer indexer = new Indexer();



    private final Turret turret = new Turret(
        () -> drivetrain.getState().Pose, 
        () -> DriverStation.getAlliance().orElse(Alliance.Red),
        () -> true
    );
    private final Turret turret;
    private final Indexer indexer;

    private final PIDController gyroController = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
    private double target = 0.0;

    public RobotContainer() {
        gyroController.enableContinuousInput(-Math.PI, Math.PI);
        gyroController.setIntegratorRange(-2.0, 2.0);

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        turret = new Turret(
            () -> drivetrain.getState().Pose, 
            () -> ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation()),
            () -> true // false when robot is climbing
        );

        indexer = new Indexer();

        configureDefaultCommands();
        configureNamedCommands();
        configureSysid();

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Rotate chassis to allow turret to shoot
        joystick.rightBumper().whileTrue(new AimChassisCommand(turret, drivetrain));

        joystick.a().onTrue(indexer.incrementSpeed(() -> true, () -> 0.1)); //TODO add logic for this stuff
        joystick.b().onTrue(indexer.incrementSpeed(()-> true, () -> -0.1));
        joystick.rightTrigger().whileTrue(new IndexerCommand(indexer, () -> true, () -> Constants.indexerConstants.PASSTHROUGH_SPEED));
        joystick.rightTrigger().whileFalse(new IndexerCommand(indexer, () -> false, () -> 0.0));
    
        joystick.rightBumper().whileTrue(new IndexerCommand(indexer, () -> 1.0));

        joystick.povUp().onTrue(new InstantCommand(() -> target = 0.0));
        joystick.povLeft().onTrue(new InstantCommand(() -> target = Math.PI/2));
        joystick.povDown().onTrue(new InstantCommand(() -> target = Math.PI));
        joystick.povRight().onTrue(new InstantCommand(() -> target = -Math.PI/2));
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("IndexerOn", new IndexerCommand(indexer, () -> 1.0));
        NamedCommands.registerCommand("IndexerOff", new IndexerCommand(indexer, () -> 0.0));
    }

    private void configureDefaultCommands() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getRightY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getRightX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(calcAutoTurn()) // Drive counterclockwise with negative X (left)
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
        // joystick.back().and(joystick.y()).whileTrue(turret.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(turret.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(turret.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(turret.sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private double calcAutoTurn() {
        if (Math.abs(joystick.getLeftX()) > 0.01) {
            target += (joystick.getLeftX() * Math.toRadians(5));
        }

        double output = gyroController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), target);

        return MathUtil.clamp(output, -MaxAngularRate, MaxAngularRate);
    }

    /** Returns the current AprilTag layout type. */
    public AprilTagLayoutType getSelectedAprilTagLayout() {
        return FieldConstants.defaultAprilTagType;
    }
}