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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PassTargetConstants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.turretTargetConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.UpdateLED;
import frc.robot.commands.IntakeJitterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PassTargetSelectorSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.RealFuelSubsystem;
import frc.robot.subsystems.SimFuelIKSubsystem;
import frc.robot.subsystems.SimFuelSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.util.FuelSim;
import frc.robot.util.HubShiftUtil;

public class RobotContainer {
        private SlewRateLimiter xLimiter = new SlewRateLimiter(10);
        private SlewRateLimiter yLimiter = new SlewRateLimiter(10);
        private SlewRateLimiter zLimiter = new SlewRateLimiter(25);

        private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private final double maxAngularRate = RotationsPerSecond.of(1.2).in(RadiansPerSecond);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(maxSpeed);
        private final CommandXboxController joystick = new CommandXboxController(0);
        private final CommandXboxController joystick2 = new CommandXboxController(1);
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private SendableChooser<Command> autoChooser;
        private final PoseEstimatorSubsystem poseEstimator;
        private final Turret turret;
        private boolean override = true;
        private final Indexer indexer;
        private final Intake intake;
        private final SimFuelSubsystem fuelSim;
        private final SimFuelIKSubsystem fuelSimIK;
        private final RealFuelSubsystem fuelReal;
        public final LEDSubsystem ledSubsystem = new LEDSubsystem();
        private final Trigger intakeHome;
        private final PassTargetSelectorSubsystem passTargetSelector;

        private final PIDController gyroController = new PIDController(Constants.gyroP, Constants.gyroI,
                        Constants.gyroD);
        private double target = drivetrain.getState().Pose.getRotation().getRadians();

        public RobotContainer() {
                gyroController.enableContinuousInput(-Math.PI, Math.PI);
                gyroController.setIntegratorRange(-2.0, 2.0);

                SmartDashboard.setDefaultBoolean(
                                AutoAimConstants.useIKSolverKey,
                                AutoAimConstants.defaultUseIKSolver);
                SmartDashboard.putBoolean(
                                AutoAimConstants.useIKSolverKey,
                                AutoAimConstants.defaultUseIKSolver);
                SmartDashboard.setDefaultBoolean(
                                AutoAimConstants.useEntryAngleIKKey,
                                AutoAimConstants.defaultUseEntryAngleIK);
                SmartDashboard.putBoolean(
                                AutoAimConstants.useEntryAngleIKKey,
                                AutoAimConstants.defaultUseEntryAngleIK);
                SmartDashboard.setDefaultBoolean(
                                AutoAimConstants.useShootOnMoveCompKey,
                                AutoAimConstants.defaultUseShootOnMoveComp);
                SmartDashboard.putBoolean(
                                AutoAimConstants.useShootOnMoveCompKey,
                                AutoAimConstants.defaultUseShootOnMoveComp);
                SmartDashboard.setDefaultNumber(
                                AutoAimConstants.modelMuzzleHeightMetersKey,
                                Constants.TurretConstants.shooterMuzzleHeightMeters);
                SmartDashboard.putNumber(
                                AutoAimConstants.modelMuzzleHeightMetersKey,
                                Constants.TurretConstants.shooterMuzzleHeightMeters);
                SmartDashboard.setDefaultNumber(
                                AutoAimConstants.modelTargetHeightMetersKey,
                                Constants.TurretConstants.targetHeightMeters);
                SmartDashboard.putNumber(
                                AutoAimConstants.modelTargetHeightMetersKey,
                                Constants.TurretConstants.targetHeightMeters);

                poseEstimator = new PoseEstimatorSubsystem(drivetrain);
                passTargetSelector = new PassTargetSelectorSubsystem();

                turret = new Turret(
                                () -> drivetrain.getState().Pose,
                                () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                                                drivetrain.getState().Speeds,
                                                drivetrain.getState().Pose.getRotation()),
                                () -> override);
                // HubShiftUtil.setTurretSupplier(() -> Optional.of(turret));

                indexer = new Indexer();
                intake = new Intake();
                intakeHome = new Trigger(() -> intake.intakeIsIn());

                fuelSim = RobotBase.isSimulation()
                                ? new SimFuelSubsystem(
                                                () -> drivetrain.getState().Pose,
                                                () -> drivetrain.getState().Speeds)
                                : null;
                fuelSimIK = RobotBase.isSimulation()
                                ? new SimFuelIKSubsystem(
                                                () -> drivetrain.getState().Pose,
                                                () -> drivetrain.getState().Speeds)
                                : null;
                fuelReal = RobotBase.isSimulation() ? null : new RealFuelSubsystem();

                configureNamedCommands();
                configureDefaultCommands();
                // configureSysid();
                configureBindings();
                configureAutoChooser();

                CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        }

        private void configureBindings() {
                joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                joystick.rightBumper().onTrue(new InstantCommand(() -> intake.useFastConfig(), intake)
                                .andThen(new InstantCommand(
                                                () -> intake.setDeployPosition(() -> IntakeConstants.deployPos),
                                                intake))
                                .withTimeout(0.25)
                                .andThen(new InstantCommand(() -> intake.setRollerPower(1), intake)));

                joystick.leftBumper().onTrue(new InstantCommand(() -> intake.useFastConfig(), intake)
                                .andThen(new InstantCommand(
                                                () -> intake.setDeployPosition(() -> IntakeConstants.homePos), intake))
                                .andThen(new InstantCommand(() -> intake.setRollerPower(0.0), intake)));

                joystick.rightTrigger()
                                .onTrue(new InstantCommand(() -> intake.setRollerPower(-1.0), intake))
                                .onFalse(new InstantCommand(() -> intake.setRollerPower(1.0), intake));

                joystick.leftTrigger()
                                .whileTrue(new IndexerCommand(indexer, () -> -0.9, () -> true))
                                .onFalse(new IndexerCommand(indexer, () -> 0.0, () -> true));

                joystick.y()
                                // .onTrue(new InstantCommand(() -> startTargeting(AutoAimConstants.defaultUseIKSolver)))
                                .whileTrue(new IndexerCommand(indexer, () -> 0.9, () -> turret.turretOnTarget()))
                                .onFalse(new IndexerCommand(indexer, () -> 0.0, () -> turret.turretOnTarget()));

                joystick.b().whileTrue(new IntakeJitterCommand(intake));

                joystick.povUp().onTrue(new InstantCommand(() -> target = Math.PI));
                joystick.povLeft().onTrue(new InstantCommand(() -> target = -(Math.PI / 2.0)));
                joystick.povDown().onTrue(new InstantCommand(() -> target = 0));
                joystick.povRight().onTrue(new InstantCommand(() -> target = Math.PI / 2.0));

                joystick2.leftTrigger().onTrue(new InstantCommand(() -> intake.setRollerPower(0.0), intake));
                joystick2.rightTrigger().whileTrue(new InstantCommand(() -> intake.useSlowConfig(), intake)
                                .andThen(new InstantCommand(
                                                () -> intake.setDeployPosition(() -> IntakeConstants.shakePos), intake))
                                .andThen(new InstantCommand(() -> intake.setRollerPower(1), intake)));
                joystick2.b().or(intakeHome)
                                .onTrue(new InstantCommand(() -> override = true))
                                .onFalse(new InstantCommand(() -> override = false));

                joystick2.y().onTrue(new InstantCommand(() -> {
                        boolean useIK = SmartDashboard.getBoolean(
                                        AutoAimConstants.useIKSolverKey,
                                        AutoAimConstants.defaultUseIKSolver);
                        SmartDashboard.putBoolean(AutoAimConstants.useIKSolverKey, !useIK);
                }));

                // Start-of-shift warning
                for (int i = 1; i <= 5; i++) {
                        double time = i;
                        Trigger shiftAboutToStart = new Trigger(
                                        () -> ((HubShiftUtil.getShiftedShiftInfo().remainingTime() < time)
                                                        && !HubShiftUtil.getShiftedShiftInfo().active()));
                        shiftAboutToStart.and(RobotModeTriggers.teleop())
                                        .onTrue(
                                                        Commands.runEnd(
                                                                        () -> {
                                                                                joystick.setRumble(
                                                                                                RumbleType.kBothRumble,
                                                                                                1.0);
                                                                                joystick2.setRumble(
                                                                                                RumbleType.kBothRumble,
                                                                                                1.0);
                                                                        },
                                                                        () -> {
                                                                                joystick.setRumble(
                                                                                                RumbleType.kBothRumble,
                                                                                                0);
                                                                                joystick2.setRumble(
                                                                                                RumbleType.kBothRumble,
                                                                                                0);
                                                                        }).withTimeout(0.25));
                }

                // End-of-shift warning
                for (int i = 1; i <= 5; i++) {
                        double time = i;
                        Trigger shiftAboutToEnd = new Trigger(
                                        () -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time));
                        shiftAboutToEnd.and(RobotModeTriggers.teleop())
                                        .onTrue(
                                                        Commands.runEnd(
                                                                        () -> {
                                                                                joystick.setRumble(
                                                                                                RumbleType.kBothRumble,
                                                                                                1.0);
                                                                                joystick2.setRumble(
                                                                                                RumbleType.kBothRumble,
                                                                                                1.0);
                                                                        },
                                                                        () -> {
                                                                                joystick.setRumble(
                                                                                                RumbleType.kBothRumble,
                                                                                                0);
                                                                                joystick2.setRumble(
                                                                                                RumbleType.kBothRumble,
                                                                                                0);
                                                                        }).withTimeout(0.5));
                }

                // Reset hub shift timer when enabling
                RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
                RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
                RobotModeTriggers.disabled().onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
                RobotModeTriggers.disabled().onTrue(Commands.runOnce(this::stopTargeting).ignoringDisable(true));

                // Auto-enable targeting in enabled modes; no button hold needed for spin-up.
                RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> enableTargeting(
                                // SmartDashboard.getBoolean(
                                // AutoAimConstants.useIKSolverKey,
                                AutoAimConstants.defaultUseIKSolver
                // )
                )));
                RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> enableTargeting(
                                // SmartDashboard.getBoolean(
                                // AutoAimConstants.useIKSolverKey,
                                AutoAimConstants.defaultUseIKSolver
                // )
                )));
        }

        public CommandSwerveDrivetrain getDrivetrain() {
                return drivetrain;
        }

        public void updateDashboard() {
                // Update from HubShiftUtil
                SmartDashboard.putString("Shifts/Remaining Shift Time",
                                String.format("%.1f",
                                                Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));
                SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
                SmartDashboard.putString("Shifts/Game State",
                                HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
                SmartDashboard.putBoolean("Shifts/Active First?",
                                DriverStation.getAlliance().orElse(Alliance.Red) == HubShiftUtil
                                                .getFirstActiveAlliance());
        }

        // private void configureSysid() {
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // joystick.start().and(joystick.a()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // joystick.start().and(joystick.b()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // }

        private void configureAutoChooser() {
                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);
        }

        private void configureNamedCommands() {
                NamedCommands.registerCommand(
                                "intake_deploy",
                                new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.deployPos),
                                                intake)
                                                .withTimeout(0.25)
                                                .andThen(new InstantCommand(() -> intake.setRollerPower(1), intake)));
                NamedCommands.registerCommand(
                                "intake_home",
                                new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.homePos),
                                                intake)
                                                .andThen(new InstantCommand(() -> intake.setRollerPower(0.0), intake)));
                NamedCommands.registerCommand(
                                "intake_shake",
                                new InstantCommand(() -> intake.useSlowConfig(), intake)
                                                .andThen(new InstantCommand(
                                                                () -> intake.setDeployPosition(
                                                                                () -> IntakeConstants.shakePos),
                                                                intake))
                                                .andThen(new InstantCommand(() -> intake.setRollerPower(1), intake)));
                NamedCommands.registerCommand(
                                "intake_wiggle",
                                new IntakeJitterCommand(intake));
                NamedCommands.registerCommand(
                                "indexer_on",
                                new IndexerCommand(indexer, () -> 0.9, () -> turret.turretOnTarget()));
                NamedCommands.registerCommand(
                                "indexer_off",
                                new IndexerCommand(indexer, () -> 0.0, () -> turret.turretOnTarget()));
                NamedCommands.registerCommand(
                                "shoot_map",
                                new InstantCommand(() -> startTargeting(false)));
                NamedCommands.registerCommand(
                                "shoot_ik",
                                new InstantCommand(() -> startTargeting(true)));
                NamedCommands.registerCommand(
                                "shoot_stop",
                                new InstantCommand(this::stopTargeting));
                NamedCommands.registerCommand(
                                "turret_stop",
                                new InstantCommand(() -> override = true));
                NamedCommands.registerCommand(
                                "turret_automatic",
                                new InstantCommand(() -> override = false));
        }

        private void configureDefaultCommands() {
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(xLimiter.calculate(-joystick.getRightY() * maxSpeed))
                                                .withVelocityY(yLimiter.calculate(-joystick.getRightX() * maxSpeed))
                                                .withRotationalRate(zLimiter.calculate(calcAutoTurn()))));

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // joystick.x().whileTrue(new DriveToPose(drivetrain, () -> new Pose2d(2, 2,
                // Rotation2d.fromDegrees(90))));
                drivetrain.registerTelemetry(logger::telemeterize);

                ledSubsystem.setDefaultCommand(new UpdateLED(ledSubsystem, poseEstimator, turret)); // startup

                // Automatically stop indexer when no button is pressed
                indexer.setDefaultCommand(new IndexerCommand(indexer, () -> 0.0, () -> turret.turretOnTarget()));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        private double calcAutoTurn() {
                if (Math.abs(joystick.getLeftX()) > 0.01) {
                        target -= joystick.getLeftX() * Math.toRadians(5);
                }

                double output = gyroController.calculate(
                                drivetrain.getState().Pose.getRotation().getRadians(),
                                target);

                return MathUtil.clamp(output, -maxAngularRate, maxAngularRate);
        }

        public AprilTagLayoutType getSelectedAprilTagLayout() {
                return FieldConstants.defaultAprilTagType;
        }

        private void enableTargeting(boolean useIK) {
                SmartDashboard.putBoolean(AutoAimConstants.useIKSolverKey, useIK);
                Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                Translation2d target = alliance == Alliance.Red
                                ? FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
                                : FieldConstants.Hub.topCenterPoint.toTranslation2d();
                SmartDashboard.putBoolean(turretTargetConstants.enableKey, true);
                SmartDashboard.putNumber(turretTargetConstants.targetXKey, target.getX());
                SmartDashboard.putNumber(turretTargetConstants.targetYKey, target.getY());
                if (fuelReal != null) {
                        fuelReal.enableTargeting(true);
                        fuelReal.setTarget(target);
                }
        }

        private void startTargeting(boolean useIK) {
                enableTargeting(useIK);

                boolean pass = SmartDashboard.getBoolean("Pass", false);
                boolean passTargetEnabled = SmartDashboard.getBoolean(
                                PassTargetConstants.enableKey, false);

                if (pass && passTargetEnabled) {
                        Translation2d target = new Translation2d(
                                        SmartDashboard.getNumber(PassTargetConstants.targetXKey,
                                                        PassTargetConstants.defaultTargetX),
                                        SmartDashboard.getNumber(PassTargetConstants.targetYKey,
                                                        PassTargetConstants.defaultTargetY));
                        if (fuelSimIK != null) {
                                Translation3d launchPosition = fuelSimIK.getRobotLaunchPosition();
                                Translation3d baseVelocity = fuelSimIK.computeLaunchVelocityToTarget(target, 0.0);
                                Translation3d launchVelocity = fuelSimIK.launchVel(baseVelocity);
                                FuelSim.getInstance().spawnFuel(launchPosition, launchVelocity);
                        }
                } else {
                        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                        Translation2d target = alliance == Alliance.Red
                                        ? FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
                                        : FieldConstants.Hub.topCenterPoint.toTranslation2d();
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
        }

        private void stopTargeting() {
                SmartDashboard.putBoolean(turretTargetConstants.enableKey, false);
                if (fuelReal != null) {
                        fuelReal.enableTargeting(false);
                }
        }
}
