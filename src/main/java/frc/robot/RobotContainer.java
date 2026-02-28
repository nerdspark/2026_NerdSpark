package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.turretTargetConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.IndexerCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RealFuelSubsystem;
import frc.robot.subsystems.SimFuelIKSubsystem;
import frc.robot.subsystems.SimFuelSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.util.FuelSim;

public class RobotContainer {
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(maxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;

    private final Turret turret;
    private final Indexer indexer;
    private final Intake intake = new Intake();
    private final SimFuelSubsystem fuelSim;
    private final SimFuelIKSubsystem fuelSimIK;
    private final RealFuelSubsystem fuelReal;

    private final PIDController gyroController =
        new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
    private double target = 0.0;

    public RobotContainer() {
        gyroController.enableContinuousInput(-Math.PI, Math.PI);
        gyroController.setIntegratorRange(-2.0, 2.0);

        SmartDashboard.setDefaultBoolean(
            AutoAimConstants.useIKSolverKey,
            AutoAimConstants.defaultUseIKSolver
        );

        turret = new Turret(
            () -> drivetrain.getState().Pose,
            () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds,
                drivetrain.getState().Pose.getRotation()),
            () -> true
        );

        indexer = new Indexer();
        fuelSim = RobotBase.isSimulation()
            ? new SimFuelSubsystem(
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds
            )
            : null;
        fuelSimIK = RobotBase.isSimulation()
            ? new SimFuelIKSubsystem(
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds
            )
            : null;
        fuelReal = RobotBase.isSimulation() ? null : new RealFuelSubsystem();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureDefaultCommands();
        configureBindings();
        configureNamedCommands();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.b().onTrue(new InstantCommand(() -> {
            boolean useIK = SmartDashboard.getBoolean(
                AutoAimConstants.useIKSolverKey,
                AutoAimConstants.defaultUseIKSolver
            );
            SmartDashboard.putBoolean(AutoAimConstants.useIKSolverKey, !useIK);
        }));

        joystick.rightTrigger()
            .whileTrue(new IndexerCommand(indexer, () -> true, () -> Constants.indexerConstants.PASSTHROUGH_SPEED))
            .whileFalse(new IndexerCommand(indexer, () -> false, () -> 0.0));

        joystick.start().onTrue(new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.deployPos), intake)
            .andThen(new InstantCommand(() -> intake.setRollerPower(1.0), intake)));
        joystick.a().onTrue(new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.homePos), intake)
            .andThen(new InstantCommand(() -> intake.setRollerPower(0.0), intake)));
        joystick.x()
            .onTrue(new InstantCommand(() -> startTargeting(true)))
            .onFalse(new InstantCommand(() -> stopTargeting()));

        joystick.povUp().onTrue(new InstantCommand(() -> target = 0.0));
        joystick.povLeft().onTrue(new InstantCommand(() -> target = Math.PI / 2.0));
        joystick.povDown().onTrue(new InstantCommand(() -> target = Math.PI));
        joystick.povRight().onTrue(new InstantCommand(() -> target = -Math.PI / 2.0));
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand(
            "intake_deploy",
            new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.deployPos), intake)
                .andThen(new InstantCommand(() -> intake.setRollerPower(1.0), intake))
        );
        NamedCommands.registerCommand(
            "intake_home",
            new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.homePos), intake)
                .andThen(new InstantCommand(() -> intake.setRollerPower(0.0), intake))
        );
        // NamedCommands.registerCommand("indexer_on", new IndexerCommand(indexer, () -> 1.0)); TODO add these back when we figure out whats wrong
        // NamedCommands.registerCommand("indexer_off", new IndexerCommand(indexer, () -> 0.0));
        NamedCommands.registerCommand("shoot_map", new InstantCommand(() -> startTargeting(false)));
        NamedCommands.registerCommand("shoot_ik", new InstantCommand(() -> startTargeting(true)));
        NamedCommands.registerCommand("shoot_stop", new InstantCommand(this::stopTargeting));
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getRightY() * maxSpeed)
                    .withVelocityY(-joystick.getRightX() * maxSpeed)
                    .withRotationalRate(calcAutoTurn())
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private double calcAutoTurn() {
        if (Math.abs(joystick.getLeftX()) > 0.01) {
            target += joystick.getLeftX() * Math.toRadians(5);
        }

        double output = gyroController.calculate(
            drivetrain.getState().Pose.getRotation().getRadians(),
            target
        );

        return MathUtil.clamp(output, -maxAngularRate, maxAngularRate);
    }

    public AprilTagLayoutType getSelectedAprilTagLayout() {
        return FieldConstants.defaultAprilTagType;
    }

    private void startTargeting(boolean useIK) {
        SmartDashboard.putBoolean(AutoAimConstants.useIKSolverKey, useIK);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d target = alliance == Alliance.Red
            ? Constants.Field.redHub
            : Constants.Field.blueHub;
        SmartDashboard.putBoolean(turretTargetConstants.enableKey, true);
        SmartDashboard.putNumber(turretTargetConstants.targetXKey, target.getX());
        SmartDashboard.putNumber(turretTargetConstants.targetYKey, target.getY());
        if (fuelReal != null) {
            fuelReal.enableTargeting(true);
            fuelReal.setTarget(target);
        }
        if (fuelSimIK != null && useIK) {
            Translation3d launchPosition = fuelSimIK.getRobotLaunchPosition();
            Translation3d baseVelocity = fuelSimIK.computeLaunchVelocityToTarget(target);
            Translation3d launchVelocity = fuelSimIK.launchVel(baseVelocity);
            FuelSim.getInstance().spawnFuel(launchPosition, launchVelocity);
        } else if (fuelSim != null && !useIK) {
            Translation3d launchPosition = fuelSim.getRobotLaunchPosition();
            Translation3d baseVelocity = fuelSim.computeLaunchVelocityToTarget(target);
            Translation3d launchVelocity = fuelSim.launchVel(baseVelocity);
            FuelSim.getInstance().spawnFuel(launchPosition, launchVelocity);
        }
    }

    private void stopTargeting() {
        SmartDashboard.putBoolean(turretTargetConstants.enableKey, false);
        if (fuelReal != null) {
            fuelReal.enableTargeting(false);
        }
    }
}
