package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

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
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.turretTargetConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.IndexerCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.RealFuelSubsystem;
import frc.robot.subsystems.SimFuelIKSubsystem;
import frc.robot.subsystems.SimFuelSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.util.FuelSim;
import frc.robot.util.HubShiftUtil;

public class RobotContainer {
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(maxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private SendableChooser<Command> autoChooser;

    private final PoseEstimatorSubsystem poseEstimator;
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

        poseEstimator = new PoseEstimatorSubsystem(drivetrain);

        turret = new Turret(
            () -> drivetrain.getState().Pose,
            () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds,
                drivetrain.getState().Pose.getRotation()),
            () -> true
        );
        HubShiftUtil.setTurretSupplier(() -> Optional.of(turret));

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

        configureNamedCommands();
        configureDefaultCommands();
        // configureSysid();
        configureBindings();
        configureAutoChooser();
        
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

        joystick.leftBumper()
            .whileTrue(new IndexerCommand(indexer, () -> true, () -> 1.0))
            .whileFalse(new IndexerCommand(indexer, () -> false, () -> 0.0));
        joystick.rightBumper().onTrue(new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.deployPos), intake)
            .andThen(new InstantCommand(() -> intake.setRollerPower(0.75), intake)));
        joystick.y().onTrue(new InstantCommand(() -> intake.setRollerPower(0.0), intake));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.homePos), intake)
            .andThen(new InstantCommand(() -> intake.setRollerPower(0.0), intake)));
        joystick.a()
            .onTrue(new InstantCommand(() -> startTargeting(true)))
            .onFalse(new InstantCommand(() -> stopTargeting()));

        joystick.povUp().onTrue(new InstantCommand(() -> target = Math.PI));
        joystick.povLeft().onTrue(new InstantCommand(() -> target = -(Math.PI / 2.0)));
        joystick.povDown().onTrue(new InstantCommand(() -> target = 0));
        joystick.povRight().onTrue(new InstantCommand(() -> target = Math.PI / 2.0));

        // Start-of-shift warning
        for (int i = 0; i < 5; i++) {
            double start = i * 0.75; // 0.25 on + 0.5 break
            double end = start + 0.25; // rumble duration

            Trigger shiftJustStarted = new Trigger(() ->
                HubShiftUtil.getShiftedShiftInfo().active()
                && HubShiftUtil.getShiftedShiftInfo().elapsedTime() > start
                && HubShiftUtil.getShiftedShiftInfo().elapsedTime() < end
            );

            shiftJustStarted.and(RobotModeTriggers.teleop())
                .onTrue(
                    Commands.runEnd(
                        () -> joystick.setRumble(RumbleType.kRightRumble, 1.0),
                        () -> joystick.setRumble(RumbleType.kBothRumble, 0.0)
                    ).withTimeout(0.25)
                );
        }

        // End-of-shift warning
        for (int i = 1; i <= 5; i++) {
            double time = i;
            Trigger shiftAboutToEnd = new Trigger(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time));
            shiftAboutToEnd.and(RobotModeTriggers.teleop())
                .onTrue(
                    Commands.runEnd(
                        () -> joystick.setRumble(RumbleType.kRightRumble, 1.0),
                        () -> joystick.setRumble(RumbleType.kBothRumble, 0.0)
                    ).withTimeout(0.25)
                );
        }

        // Reset hub shift timer when enabling
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
    }

    public void updateDashboard() {
        // Publish match time
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        // Update from HubShiftUtil
        SmartDashboard.putString("Shifts/Remaining Shift Time", 
            String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0))
        );
        SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
        SmartDashboard.putString("Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
        SmartDashboard.putBoolean("Shifts/Active First?",
            DriverStation.getAlliance().orElse(Alliance.Red) == HubShiftUtil.getFirstActiveAlliance()
        );
    }

    private void configureSysid() {
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        joystick.start().and(joystick.a()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        joystick.start().and(joystick.b()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    private void configureAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand(
            "intake_deploy",
            new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.deployPos), intake)
                .andThen(new InstantCommand(() -> intake.setRollerPower(0.75), intake))
        );
        NamedCommands.registerCommand(
            "intake_home",
            new InstantCommand(() -> intake.setDeployPosition(() -> IntakeConstants.homePos), intake)
                .andThen(new InstantCommand(() -> intake.setRollerPower(0.0), intake))
        );
        NamedCommands.registerCommand("indexer_on", new IndexerCommand(indexer, () -> true, () -> 1.0));
        NamedCommands.registerCommand("indexer_off", new IndexerCommand(indexer, () -> false, () -> 0.0));
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
            target -= joystick.getLeftX() * Math.toRadians(5);
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
            ? FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
            : FieldConstants.Hub.topCenterPoint.toTranslation2d();
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