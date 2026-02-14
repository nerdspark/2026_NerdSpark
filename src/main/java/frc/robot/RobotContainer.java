// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.TuneTurretCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RealFuelSubsystem;
import frc.robot.subsystems.SimPoseSubsystem;
import frc.robot.subsystems.SimFuelSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.util.FuelSim;
import frc.robot.Constants.Field;

public class RobotContainer {
    private static final int kSimKeyboardPort = 1;
    private static final int kShootKeyButton = 10; // Period key on DS keyboard mapping (adjust if needed)

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick simKeyboard = new Joystick(kSimKeyboardPort);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    private final Turret turret;
    private final SimFuelSubsystem fuelSim;
    private final SimPoseSubsystem simPose;
    private final RealFuelSubsystem fuelReal;

    public RobotContainer() {
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
        fuelReal = RobotBase.isSimulation()
            ? null
            : new RealFuelSubsystem();

        configureDefaultCommands();
        // configureSysid();

        configureBindings();
    }

    private void configureBindings() {
        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        if (fuelSim != null) {
            Command shootFuelCommand = new InstantCommand(this::spawnFuelToHubTarget)
                .andThen(Commands.waitSeconds(0.25))
                .repeatedly();

            joystick.x().onTrue(shootFuelCommand.until(() -> !joystick.x().getAsBoolean()));
            new Trigger(() -> simKeyboard.getRawButton(kShootKeyButton))
                .whileTrue(shootFuelCommand);
        } else if (fuelReal != null) {
            joystick.x().whileTrue(Commands.runOnce(() -> {
                fuelReal.enableTargeting(true);
                fuelReal.setHubTarget(DriverStation.getAlliance().orElse(Alliance.Blue));
            })).onFalse(Commands.runOnce(() -> fuelReal.enableTargeting(false)));
        }
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

    private void spawnFuelToHubTarget() {
        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Field.redHub
            : Field.blueHub;
        Translation3d launchPosition = fuelSim.getRobotLaunchPosition();
        Translation3d baseVelocity = fuelSim.computeLaunchVelocityToTarget(target);
        Translation3d launchVelocity = fuelSim.launchVel(baseVelocity);
        FuelSim.getInstance().spawnFuel(launchPosition, launchVelocity);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
