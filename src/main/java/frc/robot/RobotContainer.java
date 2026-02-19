// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.TurretTest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
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

    private final SendableChooser<Command> autoChooser;

    private final Turret turret;
    private final PoseEstimatorSubsystem poseEstimatorSubsystem;
    // private final Indexer indexer;

    private final Intake intake = new Intake();

    private final PIDController gyroController = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD); 
    private double target = 0.0;
    
    public RobotContainer() {
        gyroController.enableContinuousInput(-Math.PI, Math.PI);
        gyroController.setIntegratorRange(-2.0, 2.0);
        
        poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);
      
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        turret = new Turret(
            () -> drivetrain.getState().Pose, 
            () -> ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation()),
            () -> DriverStation.getAlliance().orElse(Alliance.Red),
            () -> true // false when robot is climbing
        );
        // turret = new Turret(
        //     () -> new Pose2d(), 
        //     () -> new ChassisSpeeds(),
        //     () -> DriverStation.getAlliance().orElse(Alliance.Red),
        //     () -> true // false when robot is climbing
        // );

        // indexer = new Indexer();

        configureDefaultCommands();
        configureSysid();

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // joystick.y().whileTrue(new TurretTest(turret, 85, 2.5));
        // joystick.b().whileTrue(new TurretTest(turret, 70, 2.5));
        // joystick.a().whileTrue(new TurretTest(turret, 50, 2.5));
        // joystick.x().whileTrue(new TurretTest(turret, 31, 2));

        // joystick.rightBumper().whileTrue(new IndexerCommand(indexer, () -> 1.0));

        // joystick.povUp().onTrue(new InstantCommand(() -> target = 0.0));
        // joystick.povLeft().onTrue(new InstantCommand(() -> target = Math.PI/2));
        // joystick.povDown().onTrue(new InstantCommand(() -> target = Math.PI));
        // joystick.povRight().onTrue(new InstantCommand(() -> target = -Math.PI/2));
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

       // joystick.x().whileTrue(new DriveToPose(drivetrain, () -> new Pose2d(2, 2, Rotation2d.fromDegrees(90))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        
        joystick.a().onTrue(
            new InstantCommand(
                () -> intake.setDeployPosition(() -> IntakeConstants.deployPos),
                intake
            ).andThen(
                new InstantCommand(
                    () -> intake.setRollerPower(1),
                    intake
                )
            )
        );

        joystick.x().onTrue(
            new InstantCommand(
                () -> intake.setDeployPosition(() -> IntakeConstants.homePos),
                intake
            ).andThen(
                new InstantCommand(
                    () -> intake.setRollerPower(0.0),
                    intake
                )
            )
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
        // return null;
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