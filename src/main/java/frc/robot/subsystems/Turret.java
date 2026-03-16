package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.TurretUtil.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.ShooterParams;
import frc.robot.Constants.TurretConfig;
import frc.robot.Constants;
import frc.robot.Constants.turretTargetConstants;
import frc.robot.FieldConstants;
import frc.robot.Constants.ShootMode;
import frc.robot.Constants.IkSolution;
import frc.robot.Constants.MapTuneConstants;
import frc.robot.Constants.SlippageCorrectionConstants;
import frc.robot.util.ShooterOffsetMap;
import frc.robot.util.SlippageCorrectionMap;

public class Turret extends SubsystemBase {
    private static final double TWO_PI = 2.0 * Math.PI;
    
    private CANBus canivore;
    private TalonFX spinMotor, hoodMotor1, hoodMotor2, shootMotor1, shootMotor2;
    private CANcoder spinCancoder1, spinCancoder2;

    private VelocityDutyCycle shootDutyBang = new VelocityDutyCycle(0);
    private VelocityTorqueCurrentFOC shootTorqueBang = new VelocityTorqueCurrentFOC(0);
    private MotionMagicTorqueCurrentFOC hoodPose = new MotionMagicTorqueCurrentFOC(0);
    private MotionMagicVoltage spinPose = new MotionMagicVoltage(0);

    // private VoltageOut sysId = new VoltageOut(0);

    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speed;
    private Supplier<Boolean> manualOverride;

    private double motorPositon;
    private double turretAngle = 0;
    private boolean brake = false;

    private double filteredTurretDelaySec = TurretConstants.delay;
    public static double delaySum = 0.0;
    public static int delaySamples = 0;
    public static double maxDelay = 0.0;


    private Debouncer torqueCurrentDebouncer = new Debouncer(0.02, DebounceType.kFalling);
    private ShootMode mode = ShootMode.COAST;
    private double velocity = 0;
    public double tof = 0;

    private final Field2d m_field = new Field2d();
    private final ShooterOffsetMap offsetMap = new ShooterOffsetMap();
    private final SlippageCorrectionMap slippageMap = new SlippageCorrectionMap();

    public Turret(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds, Supplier<Boolean> manualOverrider) {
        pose = robotPose;
        speed = speeds;
        manualOverride = manualOverrider;

        canivore = new CANBus(Constants.CANbus);
        
        spinMotor = new TalonFX(TurretConfig.spinMotorId, canivore);
        hoodMotor1 = new TalonFX(TurretConfig.hoodMotor1Id, canivore);
        hoodMotor2 = new TalonFX(TurretConfig.hoodMotor2Id, canivore);
        shootMotor1 = new TalonFX(TurretConfig.shootMotor1Id, canivore);
        shootMotor2 = new TalonFX(TurretConfig.shootMotor2Id, canivore);

        spinCancoder1 = new CANcoder(TurretConfig.spinCancoder1Id, canivore);
        spinCancoder2 = new CANcoder(TurretConfig.spinCancoder2Id, canivore);

        TalonFXConfiguration spinConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.spinKp)
                .withKI(TurretConfig.spinKi)
                .withKD(TurretConfig.spinKd)
                .withKS(TurretConfig.spinKs)
                .withKV(TurretConfig.spinKv)
                .withKA(TurretConfig.spinKa)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.spinStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(TurretConfig.spinSupplyCurrent)
                .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(TurretConfig.spinVelocity)
                .withMotionMagicAcceleration(TurretConfig.spinAccel)
            )
        ;
        TalonFXConfiguration hoodConfig1 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.hoodKp1)
                .withKI(TurretConfig.hoodKi1)
                .withKD(TurretConfig.hoodKd1)
                .withKS(TurretConfig.hoodKs1)
                .withKV(TurretConfig.hoodKv1)
                .withKA(TurretConfig.hoodKa1)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.hoodStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(TurretConfig.hoodSupplyCurrentLimit))
                .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(TurretConfig.hoodVelocity)
                .withMotionMagicAcceleration(TurretConfig.hoodAccel)
            )
        ;
        TalonFXConfiguration hoodConfig2 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.hoodKp2)
                .withKI(TurretConfig.hoodKi2)
                .withKD(TurretConfig.hoodKd2)
                .withKS(TurretConfig.hoodKs2)
                .withKV(TurretConfig.hoodKv2)
                .withKA(TurretConfig.hoodKa2)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.hoodStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(TurretConfig.hoodSupplyCurrentLimit))
                .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(TurretConfig.hoodVelocity)
                .withMotionMagicAcceleration(TurretConfig.hoodAccel)
            )
        ;
        TalonFXConfiguration shootConfig1 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withPeakForwardDutyCycle(TurretConfig.peakDutyCycle)
                .withPeakReverseDutyCycle(0)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.bangbangKp)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(TurretConfig.peakTorque)
                .withPeakReverseTorqueCurrent(0)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.shootStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(TurretConfig.shootSupplyCurrentLimit))
                .withSupplyCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration shootConfig2 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive)
                .withPeakForwardDutyCycle(TurretConfig.peakDutyCycle)
                .withPeakReverseDutyCycle(0)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.bangbangKp)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(TurretConfig.peakTorque)
                .withPeakReverseTorqueCurrent(0)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.shootStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(TurretConfig.shootSupplyCurrentLimit))
                .withSupplyCurrentLimitEnable(true)
            )
        ;
        CANcoderConfiguration spinCancoder1Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withMagnetOffset(TurretConfig.spinCancoder1Offset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            )
        ;
        CANcoderConfiguration spinCancoder2Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withMagnetOffset(TurretConfig.spinCancoder2Offset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            )
        ;

        spinMotor.getConfigurator().apply(spinConfig);
        hoodMotor1.getConfigurator().apply(hoodConfig1);
        hoodMotor2.getConfigurator().apply(hoodConfig2);
        shootMotor1.getConfigurator().apply(shootConfig1);
        shootMotor2.getConfigurator().apply(shootConfig2);

        spinCancoder1.getConfigurator().apply(spinCancoder1Config);
        spinCancoder2.getConfigurator().apply(spinCancoder2Config);

        hoodMotor2.setControl(new Follower(TurretConfig.hoodMotor1Id, MotorAlignmentValue.Opposed));
        shootMotor2.setControl(new Follower(TurretConfig.shootMotor1Id, MotorAlignmentValue.Opposed));

        hoodMotor1.setPosition(0);
        hoodMotor2.setPosition(0);

        double theta1 = spinCancoder1.getAbsolutePosition().getValueAsDouble() * TWO_PI;
        double theta2 = spinCancoder2.getAbsolutePosition().getValueAsDouble() * TWO_PI;
        
        double turretAngle = floorMod(
            (TWO_PI/TurretConstants.spinTeeth) * (
                floorMod(
                    -(TurretConstants.spinCancoder1Teeth/TWO_PI) * theta1, TurretConstants.spinCancoder1Teeth
                ) + TurretConstants.spinCancoder1Teeth * floorMod(
                    modInverse(
                        TurretConstants.spinCancoder1Teeth, TurretConstants.spinCancoder2Teeth
                    ) * (
                        floorMod(-(TurretConstants.spinCancoder2Teeth/TWO_PI) * theta2, TurretConstants.spinCancoder2Teeth) 
                        - floorMod(-(TurretConstants.spinCancoder1Teeth/TWO_PI) * theta1, TurretConstants.spinCancoder1Teeth)
                    ), 
                    TurretConstants.spinCancoder2Teeth
                )
            ), 
            TWO_PI
        );
        // Normalize to -pi to pi
        motorPositon = (normalizeRadians(turretAngle) * TurretConstants.spinRatio) / TWO_PI;
        spinMotor.setPosition(motorPositon, 2.5);

        initMapTuneDashboard();
    }

    // private final SysIdRoutine spin = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null, // Use default ramp rate (1 V/s)
    //         Volts.of(6), // Reduce dynamic step voltage to 5 V to prevent brownout
    //         null, // Use 5s timeout
    //         state -> SignalLogger.writeString("SysIdSpin_State", state.toString())
    //     ), 
    //     new SysIdRoutine.Mechanism(
    //         output -> spinMotor.setControl(sysId.withOutput(output)),
    //         null,
    //         this
    //     )
    // );

    // private SysIdRoutine sysIdRoutineToApply = spin;

   /**
     * Estimates and filters turret phase delay in seconds. Updates the filtered phase delay in seconds
     *
     * @param desiredAngleRad   Desired turret angle (radians, normalized)
     * @param currentAngleRad   Current turret angle (radians, normalized)
     * @param motorVelRadPerSec Measured turret motor angular velocity (rad/s, signed)
     * @param dtSec             Loop period in seconds
     */
    private void estimateTurretPhaseDelaySec(double desiredAngleRad, double currentAngleRad, double motorVelRadPerSec, 
        double dtSec) {
        if (!DriverStation.isDisabled()) {
            /* ---------------- Raw delay estimate ---------------- */
            // Shortest angular error
            double error = MathUtil.angleModulus(desiredAngleRad - currentAngleRad);

            // Prevent divide-by-zero
            double effectiveVel = Math.max(Math.abs(motorVelRadPerSec), 0.0001);

            double rawDelaySec = Math.abs(error) / effectiveVel;

            rawDelaySec = MathUtil.clamp(rawDelaySec, 0.0, TurretConstants.maxDelay);

            /* ---------------- Asymmetric filter ---------------- */
            double alpha;
            if (rawDelaySec > filteredTurretDelaySec) {
                alpha = dtSec / TurretConstants.riseTime;
            } else {
                alpha = dtSec / TurretConstants.fallTime;
            }

            alpha = MathUtil.clamp(alpha, 0.0, 1.0);

            filteredTurretDelaySec += alpha * (rawDelaySec - filteredTurretDelaySec);
        }
    }

    /**
     * Calculates the dynamic X threshold line for a signal state (shoot or pass),
     * taking into account velocity and direction.
     *
     * @param startingLine      Original X coordinate of the line
     * @param robotX            Current robot X position
     * @param robotVeloX        Current robot X velocity
     * @param safetyMargin      Extra distance to start retracting early
     * @param approachDirection The sign of the robot velo going toward the line
     * @return X value of dynamic trigger line
     */
    private double calcTriggerLine(double startingLine, double robotX, double robotVeloX, double safetyMargin, boolean goingLeft) {
        if (goingLeft) {
            // Only extend the line if moving toward it
            if (robotVeloX <= 0.01) {
                // Moving away or stopped, use original line
                return startingLine - safetyMargin;
            }

            // Distance traveled while retracting
            double preTriggerDistance = (-robotVeloX * TurretConstants.hoodRetractTime) - safetyMargin;

            // Move the line backward along approach direction
            return startingLine + preTriggerDistance;
        } else {
            // Only extend the line if moving toward it
            if (-robotVeloX <= 0.01) {
                // Moving away or stopped, use original line
                return startingLine + safetyMargin;
            }

            // Distance traveled while retracting
            double preTriggerDistance = (robotVeloX * TurretConstants.hoodRetractTime) - safetyMargin;

            // Move the line backward along approach direction
            return startingLine - preTriggerDistance;
        }
    }

    /** 
     * Aims the hood of the turret and spins wheels based on shooter map and chassis speeds
     * If hood is not tight then look into chassis velocity based correction for hood
     * 
     * @param distance the distance to the center of the hub from the center of the robot
     * @param robotHeading the curret heading of the robot
     * @param robotFOS the current field centric speeds of the robot
     * @return the velocity to shoot at
    */
    private double aimOnFly(double distance, double robotHeading, ChassisSpeeds robotFOS) {
        ShooterParams map = TurretConstants.map.get(distance);

        hoodPose.Position = map.hoodPose;

        // Field heading of shooter (radians)
        double shooterFOA = robotHeading + turretAngle;

        // Compute velocity component parallel to shooter FOA using field-frame speeds
        // v_parallel = vx * cos(shooterFOA) + vy * sin(shooterFOA)
        double vParallel = robotFOS.vxMetersPerSecond * Math.cos(shooterFOA) 
                         + robotFOS.vyMetersPerSecond * Math.sin(shooterFOA);
        double deltaMotorRPS =  vParallel / (TWO_PI * TurretConstants.shooterWheelRadius);
        SmartDashboard.putNumber("Turret/debug/vParallel", vParallel);
        SmartDashboard.putNumber("Turret/debug/deltaMotorRps", deltaMotorRPS);

        double velo =  map.shooterSpeed - deltaMotorRPS;

        return applyShooterControl(velo);
    }

    /**
     * When we are out of shooting range stop wheels and send hood to stow
     */
    private void hoodWheelsZero() {
        hoodPose.Position = 0;
        mode = ShootMode.COAST;
    }

    /**
    * Aims the turret only
    *
    * @param neededAngle the field-centric target angle minus the chassis heading in radians
    */
    private void aimTurret(double neededAngle) {
        motorPositon = spinMotor.getPosition().getValueAsDouble();
        turretAngle = (motorPositon * TWO_PI) / TurretConstants.spinRatio;
        SmartDashboard.putNumber("Turret Angle", Math.toDegrees(turretAngle));

        neededAngle = normalizeRadians(neededAngle - Math.toRadians(145));
        neededAngle = Math.round(neededAngle * 100.0) / 100.0;
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(neededAngle));

        if (!brake) {
            // Update phase delay here, 20ms loop
            estimateTurretPhaseDelaySec(
                neededAngle, 
                turretAngle, 
                spinMotor.getVelocity().getValueAsDouble() * TWO_PI, 
                0.02
            );
        }

        double motorRots = (neededAngle * TurretConstants.spinRatio) / TWO_PI;

        brake = Math.abs(motorRots - spinMotor.getPosition().getValueAsDouble()) <= 0.01389;

        spinPose.Position = motorRots;
    }

    public boolean turretOnTarget() {
        return Math.abs(spinPose.Position - spinMotor.getPosition().getValueAsDouble()) < 0.4167;
    }

    private double applyShooterControl(double motorRps) {
        if (motorRps < 5) {
            mode = ShootMode.COAST;
            return 0;
        }

        boolean inTolerance = Math.abs(shootMotor1.getVelocity().getValueAsDouble() - motorRps) <= 3;
        boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
        mode = torqueCurrentControl ? ShootMode.TORQUE_CURRENT_BANG_BANG : ShootMode.DUTY_CYCLE_BANG_BANG;
        SmartDashboard.putString("Shoot Mode", mode.toString());

        return motorRps;
    }

    private double getConfiguredMuzzleHeightMeters() {
        return SmartDashboard.getNumber(
            AutoAimConstants.modelMuzzleHeightMetersKey,
            TurretConstants.shooterMuzzleHeightMeters
        );
    }

    private double getConfiguredTargetHeightMeters() {
        return SmartDashboard.getNumber(
            AutoAimConstants.modelTargetHeightMetersKey,
            TurretConstants.targetHeightMeters
        );
    }
    // TODO
    /**
     * Return value of applyChassisVelocityComp.
     * motorRps            – slippage-corrected wheel speed to command
     * turretAngleOffsetRad – how far the turret must lead the geometric aim point
     *                        so the ball's field-frame velocity points at the target
     *                        (non-zero when the robot has a perpendicular velocity component)
     */
    private static final class SotmResult {
        private final double motorRps;
        private final double turretAngleOffsetRad;
        private SotmResult(double motorRps, double turretAngleOffsetRad) {
            this.motorRps = motorRps;
            this.turretAngleOffsetRad = turretAngleOffsetRad;
        }
    }

    private static final class DirectIkSelection {
        private final double hoodDegrees;
        private final double motorRps;
        private final boolean usedPrimaryObjective;

        private DirectIkSelection(double hoodDegrees, double motorRps, boolean usedPrimaryObjective) {
            this.hoodDegrees = hoodDegrees;
            this.motorRps = motorRps;
            this.usedPrimaryObjective = usedPrimaryObjective;
        }
    }

    private IkSolution solveIK(double distanceMeters) {
        if (distanceMeters <= 0.0) {
            return null;
        }
        boolean useEntryAngleIK = SmartDashboard.getBoolean(
            AutoAimConstants.useEntryAngleIKKey,
            AutoAimConstants.defaultUseEntryAngleIK
        );
        double deltaHeight = getConfiguredTargetHeightMeters() - getConfiguredMuzzleHeightMeters();

        DirectIkSelection selection = useEntryAngleIK
            ? solveEntryAngleIKDirect(distanceMeters, deltaHeight)
            : solveMinimumSpeedIKDirect(distanceMeters, deltaHeight);
        if (selection == null) {
            return null;
        }

        ShooterOffsetMap.Offsets offsets = offsetMap.sample(distanceMeters);
        double hoodDeg = Math.max(
            TurretConstants.hoodMinDegrees,
            Math.min(
                TurretConstants.hoodMaxDegrees,
                selection.hoodDegrees + offsets.hoodOffsetDeg
            )
        );
        double theoreticalMotorRps = Math.max(
            0.0,
            useEntryAngleIK
                ? Math.min(TurretConstants.shooterMaxMotorRps, selection.motorRps + offsets.motorRpsOffset)
                : selection.motorRps + offsets.motorRpsOffset
        );
        double slippageFactor = slippageMap.efficiencyAt(theoreticalMotorRps);
        double motorRps = slippageMap.correctedMotorRps(theoreticalMotorRps);
        // After slippage correction the commanded RPS can exceed the motor's physical
        // limit even if the theoretical RPS was within bounds — reject the solution.
        if (motorRps > TurretConstants.shooterMaxMotorRps) {
            return null;
        }
        SmartDashboard.putBoolean("Turret/IK/UseEntryAngleMode", useEntryAngleIK);
        SmartDashboard.putBoolean("Turret/IK/UsingEntryBand", selection.usedPrimaryObjective);
        SmartDashboard.putString(
            "Turret/IK/SolverMode",
            useEntryAngleIK ? "EntryAngle" : "MinimumSpeed"
        );
        SmartDashboard.putNumber("Turret/Slippage/TheoreticalMotorRps", theoreticalMotorRps);
        SmartDashboard.putNumber("Turret/Slippage/CorrectedMotorRps", motorRps);
        SmartDashboard.putNumber("Turret/Slippage/EfficiencyFactor", slippageFactor);
        return new IkSolution(hoodDeg, motorRps);
    }

    private DirectIkSelection solveEntryAngleIKDirect(
        double distanceMeters,
        double deltaHeightMeters
    ) {
        double targetEntryRad = Math.toRadians(TurretConstants.ikEntryAngleTargetDeg);
        double desiredThetaRad = Math.atan(Math.tan(targetEntryRad) + (2.0 * deltaHeightMeters / distanceMeters));
        double desiredThetaDeg = Math.toDegrees(desiredThetaRad);
        if (!Double.isFinite(desiredThetaDeg)) {
            return solveMinimumSpeedIKDirect(distanceMeters, deltaHeightMeters);
        }
        if (desiredThetaDeg < TurretConstants.hoodMinDegrees || desiredThetaDeg > TurretConstants.hoodMaxDegrees) {
            return solveMinimumSpeedIKDirect(distanceMeters, deltaHeightMeters);
        }
        double speedMps = solveIKSpeed(distanceMeters, Math.toRadians(desiredThetaDeg), deltaHeightMeters);
        double motorRps = launchSpeedMpsToMotorRps(speedMps);
        if (Double.isFinite(motorRps) && motorRps > 0.0 && motorRps <= TurretConstants.shooterMaxMotorRps) {
            return new DirectIkSelection(desiredThetaDeg, motorRps, true);
        }

        return solveMinimumSpeedIKDirect(distanceMeters, deltaHeightMeters);
    }

    private DirectIkSelection solveMinimumSpeedIKDirect(
        double distanceMeters,
        double deltaHeightMeters
    ) {
        double alphaRad = Math.atan2(deltaHeightMeters, distanceMeters);
        double thetaDeg = 90 - Math.toDegrees(0.5 * (alphaRad + (Math.PI / 2.0)));
        double thetaSpeed = Math.toDegrees(0.5 * (alphaRad + (Math.PI / 2.0)));
        if (thetaDeg < TurretConstants.hoodMinDegrees || thetaDeg > TurretConstants.hoodMaxDegrees) {
            return null;
        }
        double speedMps = solveIKSpeed(distanceMeters, Math.toRadians(thetaSpeed), deltaHeightMeters);
        double motorRps = launchSpeedMpsToMotorRps(speedMps);
        if (!Double.isFinite(motorRps) || motorRps <= 0.0 || motorRps > TurretConstants.shooterMaxMotorRps) {
            return null;
        }
        return new DirectIkSelection(thetaDeg, motorRps, false);
    }

    private double solveIKSpeed(double distanceMeters, double thetaRad, double deltaHeightMeters) {
        double cos = Math.cos(thetaRad);
        if (Math.abs(cos) < 1e-6) {
            return Double.NaN;
        }
        double tan = Math.tan(thetaRad);
        double denominator = 2.0 * cos * cos * (distanceMeters * tan - deltaHeightMeters);
        if (denominator <= 0.0) {
            return Double.NaN;
        }
        double vSquared = (9.80665 * distanceMeters * distanceMeters) / denominator;
        if (vSquared <= 0.0) {
            return Double.NaN;
        }
        return Math.sqrt(vSquared);
    }

    private double computeEntryAngleDeg(double distanceMeters, double launchSpeedMps, double launchAngleRad) {
        double horizontalSpeed = launchSpeedMps * Math.cos(launchAngleRad);
        if (horizontalSpeed <= 1e-6) {
            return Double.NaN;
        }
        double time = distanceMeters / horizontalSpeed;
        if (!Double.isFinite(time) || time <= 0.0) {
            return Double.NaN;
        }
        double verticalVelocityAtTarget = launchSpeedMps * Math.sin(launchAngleRad) - 9.80665 * time;
        return Math.toDegrees(Math.atan2(-verticalVelocityAtTarget, horizontalSpeed));
    }
    // TODO
    /**
     * Vector-based shoot-on-the-move compensation.
     *
     * The IK solver finds the exit speed and hood angle needed to reach the lead
     * target as if the robot were stationary.  When the robot is moving, the ball
     * inherits the robot's velocity, so the wheel and turret must be adjusted so
     * that (barrel velocity + robot velocity) equals the desired field velocity.
     *
     * Steps:
     *  1. Project the desired field-frame exit velocity onto the horizontal plane.
     *  2. Subtract the robot's velocity to get the required barrel velocity vector.
     *  3. The magnitude of that vector determines the new wheel speed.
     *  4. The angle difference between the barrel vector and the geometric aim
     *     direction is the turret lead offset.
     *  5. Apply slippage correction to the new theoretical RPS before returning.
     *
     * @param exitSpeedMps    theoretical exit speed from the IK solver (m/s)
     * @param launchAngleRad  physics launch angle from horizontal (radians) — thetaSpeed
     * @param aimDirectionRad field-frame direction from robot to lead target (radians)
     * @param robotFOS        current field-relative robot velocity
     * @return SotmResult containing the corrected motor RPS and turret angle offset
     */
    private SotmResult applyChassisVelocityComp(
        double exitSpeedMps,
        double launchAngleRad,
        double aimDirectionRad,
        ChassisSpeeds robotFOS
    ) {
        // Convert the IK exit speed to a theoretical motor RPS (before slippage)
        double theoreticalRps = launchSpeedMpsToMotorRps(exitSpeedMps);
        boolean useShootOnMoveComp = SmartDashboard.getBoolean(
            AutoAimConstants.useShootOnMoveCompKey,
            AutoAimConstants.defaultUseShootOnMoveComp
        );
        // TODO
        // If SOTM is disabled, just apply slippage correction with no turret offset
        if (!useShootOnMoveComp) {
            return new SotmResult(slippageMap.correctedMotorRps(theoreticalRps), 0.0);
        }

        // Step 1: horizontal component of the desired exit velocity in the field frame.
        // cos(launchAngle) separates the total exit speed into its horizontal component.
        double exitHorizSpeed = exitSpeedMps * Math.cos(launchAngleRad);
        // Decompose into X/Y using the geometric aim direction (toward lead target)
        double desiredVx = exitHorizSpeed * Math.cos(aimDirectionRad);
        double desiredVy = exitHorizSpeed * Math.sin(aimDirectionRad);

        // Step 2: subtract robot velocity to get what the barrel must actually produce.
        // Without this the robot's motion adds to or subtracts from the ball's speed
        // and deflects it sideways, causing misses.
        // SotmVelocityScale trims the robot velocity contribution:
        //   >1.0 = treats robot as moving faster → less RPS while moving (shots landing short → increase above 1)
        //   <1.0 = treats robot as moving slower → more RPS while moving  (shots landing long  → decrease below 1)
        double sotmVelScale = SmartDashboard.getNumber(
            SlippageCorrectionConstants.sotmVelocityScaleKey,
            SlippageCorrectionConstants.defaultSotmVelocityScale
        );
        double barrelVx = desiredVx - robotFOS.vxMetersPerSecond * sotmVelScale;
        double barrelVy = desiredVy - robotFOS.vyMetersPerSecond * sotmVelScale;

        // Step 3: magnitude and direction of the required barrel velocity vector
        double barrelHorizSpeed = Math.hypot(barrelVx, barrelVy);
        double barrelDirectionRad = Math.atan2(barrelVy, barrelVx);

        // Step 4: how far the turret must offset from the geometric aim direction
        // so the barrel points along the corrected velocity vector
        double turretAngleOffsetRad = normalizeRadians(barrelDirectionRad - aimDirectionRad);

        // Step 5: scale total exit speed — the barrel horizontal speed must equal
        // barrelHorizSpeed, so divide back through by cos(launchAngle) to recover
        // the required total (3-D) exit speed, then convert and apply slippage.
        double cosLaunch = Math.cos(launchAngleRad);
        double newExitSpeedMps = (Math.abs(cosLaunch) < 1e-6) ? exitSpeedMps : barrelHorizSpeed / cosLaunch;

        double newTheoreticalRps = launchSpeedMpsToMotorRps(newExitSpeedMps);
        double correctedRps = slippageMap.correctedMotorRps(newTheoreticalRps);

        SmartDashboard.putNumber("Turret/SOTM/ExitHorizSpeed", exitHorizSpeed);
        SmartDashboard.putNumber("Turret/SOTM/BarrelHorizSpeed", barrelHorizSpeed);
        SmartDashboard.putNumber("Turret/SOTM/TurretOffsetDeg", Math.toDegrees(turretAngleOffsetRad));
        SmartDashboard.putNumber("Turret/SOTM/DeltaTheoreticalRps", theoreticalRps - newTheoreticalRps);
        return new SotmResult(correctedRps, turretAngleOffsetRad);
    }

    private void initMapTuneDashboard() {
        SmartDashboard.setDefaultBoolean(MapTuneConstants.enableKey, MapTuneConstants.defaultEnable);
        SmartDashboard.setDefaultNumber(MapTuneConstants.hoodKey, 0);
        SmartDashboard.setDefaultNumber(MapTuneConstants.shooterKey, 0);
    }

    private void applyLiveMap() {
        hoodPose.Position = SmartDashboard.getNumber(MapTuneConstants.hoodKey, 0);
        velocity = SmartDashboard.getNumber(MapTuneConstants.shooterKey, 0);
        applyShooterControl(velocity);
    }

    @Override
    public void periodic() {
        ChassisSpeeds speeds = speed.get();
        Pose2d currPose = pose.get();
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currPose.getRotation());
        m_field.setRobotPose(currPose);
        Pose2d delayPose = currPose.exp(new Twist2d( // Account for phase delay
            robotSpeeds.vxMetersPerSecond * filteredTurretDelaySec, 
            robotSpeeds.vyMetersPerSecond * filteredTurretDelaySec,
            robotSpeeds.omegaRadiansPerSecond * filteredTurretDelaySec
        ));
        SmartDashboard.putNumber("Phase Delay", filteredTurretDelaySec);
        // m_field.getObject("Delay Pose").setPose(delayPose);
        Translation2d rotationOffset = TurretConstants.robotToTurret.rotateBy(currPose.getRotation()); // TODO CHANGE TO DELAYPOSE
        Pose2d turretPose = new Pose2d(currPose.getTranslation().plus(rotationOffset), currPose.getRotation()); // TODO CHANGE TO DELAYPOSE

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
        SmartDashboard.putBoolean("Is Blue", isBlue);
        double shootLine = calcTriggerLine(
            isBlue ? FieldConstants.LinesVertical.blueShootLine : FieldConstants.LinesVertical.redShootLine, 
            turretPose.getX(), 
            speeds.vxMetersPerSecond, 
            Units.inchesToMeters(20), 
            isBlue ? true : false
        );
        double passLine = calcTriggerLine(
            isBlue ? FieldConstants.LinesVertical.bluePassLine : FieldConstants.LinesVertical.redPassLine, 
            turretPose.getX(), 
            speeds.vxMetersPerSecond, 
            Units.inchesToMeters(20), 
            isBlue ? false : true
        );

        boolean shoot = isBlue ? turretPose.getX() < shootLine : turretPose.getX() > shootLine;
        SmartDashboard.putBoolean("Shoot", shoot);
        boolean pass = isBlue ? turretPose.getX() > passLine : turretPose.getX() < passLine; 
        SmartDashboard.putBoolean("Pass", pass);

        if (shoot || pass) {
            delaySum += filteredTurretDelaySec;
            delaySamples++;
            maxDelay = Math.max(maxDelay, filteredTurretDelaySec);

            boolean forceTarget = SmartDashboard.getBoolean(
                turretTargetConstants.enableKey,
                turretTargetConstants.defaultEnable
            );
            forceTarget = false;

            Translation2d goalPose;
            Translation2d passPose;
            if (isBlue) {
                goalPose = FieldConstants.Hub.topCenterPoint.toTranslation2d();
                if (forceTarget) {
                    double targetX = SmartDashboard.getNumber(
                        turretTargetConstants.targetXKey,
                        turretTargetConstants.defaultTargetX
                    );
                    double targetY = SmartDashboard.getNumber(
                        turretTargetConstants.targetYKey,
                        turretTargetConstants.defaultTargetY
                    );

                    passPose = new Translation2d(targetX, targetY);
                } else {
                    passPose = closerPoint(turretPose, FieldConstants.LeftBump.nearLeftCorner, FieldConstants.RightBump.nearLeftCorner) 
                        ? FieldConstants.LeftBump.nearLeftCorner : FieldConstants.RightBump.nearLeftCorner;
                }
            } else {
                goalPose = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
                if (forceTarget) {
                    double targetX = SmartDashboard.getNumber(
                        turretTargetConstants.targetXKey,
                        turretTargetConstants.defaultTargetX
                    );
                    double targetY = SmartDashboard.getNumber(
                        turretTargetConstants.targetYKey,
                        turretTargetConstants.defaultTargetY
                    );

                    passPose = new Translation2d(targetX, targetY);
                } else {
                    passPose = closerPoint(turretPose, FieldConstants.LeftBump.oppFarLeftCorner, FieldConstants.RightBump.oppNearLeftCorner) 
                        ? FieldConstants.LeftBump.oppFarLeftCorner : FieldConstants.RightBump.oppNearLeftCorner;
                }
            }
            
            Translation2d targetPose = shoot ? goalPose : passPose;
            m_field.getObject("Target Pose").setPose(targetPose.getMeasureX(), targetPose.getMeasureY(), new Rotation2d());
            double distance = turretPose.getTranslation().getDistance(targetPose);

            boolean useIK = SmartDashboard.getBoolean(
                AutoAimConstants.useIKSolverKey,
                AutoAimConstants.defaultUseIKSolver
            );

            if (useIK) {
                IkSolution solution = solveIK(distance);
                if (solution != null) {
                    tof = tofFromIK(solution.motorRps, solution.hoodDegrees, distance);
                } else {
                    tof = tofFromMap(TurretConstants.map.get(distance), distance);
                }
            } else {
                tof = tofFromMap(TurretConstants.map.get(distance), distance);
            }
            Translation2d lookaheadTurretPos = turretPose.getTranslation();

            for (int i = 0; i < 40; i++) {
                Translation2d fieldVelocity = new Translation2d(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond
                );
                SmartDashboard.putNumber("Turret/debug/fieldVelocityX", fieldVelocity.getX()); // after rename
                SmartDashboard.putNumber("Turret/debug/fieldVelocityY", fieldVelocity.getY());
                Translation2d flightOffset = fieldVelocity.times(tof); // How far robot moves during ball flight
                SmartDashboard.putNumber("Turret/debug/flightOffsetX", flightOffset.getX());
                SmartDashboard.putNumber("Turret/debug/flightOffsetY", flightOffset.getY());
                lookaheadTurretPos = turretPose.getTranslation().plus(flightOffset); // Effective launch point
                distance = lookaheadTurretPos.getDistance(targetPose); // Recompute distance
                if (useIK) { // Recompute TOF for new distance
                    IkSolution solution = solveIK(distance);
                    if (solution != null) {
                        tof = tofFromIK(solution.motorRps, solution.hoodDegrees, distance);
                    } else {
                        tof = tofFromMap(TurretConstants.map.get(distance), distance);
                    }
                } else {
                    tof = tofFromMap(TurretConstants.map.get(distance), distance);
                }
            }
            m_field.getObject("Look Ahead Pose").setPose(lookaheadTurretPos.getMeasureX(), lookaheadTurretPos.getMeasureY(), new Rotation2d());

            double xError = targetPose.getX() - lookaheadTurretPos.getX();
            double yError = targetPose.getY() - lookaheadTurretPos.getY();
            double errorDegrees = Math.atan2(yError, xError);
            SmartDashboard.putNumber("Turret/debug/neededDeg", Math.toDegrees(normalizeRadians(errorDegrees - turretPose.getRotation().getRadians())));
            distance = Math.hypot(yError, xError);
            SmartDashboard.putNumber("Turret/DistanceToTarget", distance);
                        
            if (useIK) {
                IkSolution ikSolution = solveIK(distance);
                double ikCompensatedMotorRps = Double.NaN;
                double turretAngleOffsetRad = 0.0;
                if (ikSolution != null) {
                    // TODO
                    // hoodDegrees is the mechanical hood angle (complement of launch angle),
                    // so the physics launch angle = 90 - hoodDegrees
                    double launchAngleRad = Math.toRadians(90.0 - ikSolution.hoodDegrees);
                    double deltaHeight = getConfiguredTargetHeightMeters() - getConfiguredMuzzleHeightMeters();
                    // Re-derive the theoretical exit speed from IK geometry (before slippage)
                    // so SOTM math works in physical velocity space, not slippage-corrected RPS
                    double exitSpeedMps = solveIKSpeed(distance, launchAngleRad, deltaHeight);
                    // Compute the barrel velocity vector that, combined with robot motion,
                    // produces the correct field-frame ball velocity toward the lead target.
                    // Returns slippage-corrected RPS and the turret angle offset to apply.
                    SotmResult sotm = applyChassisVelocityComp(exitSpeedMps, launchAngleRad, errorDegrees, speeds);
                    ikCompensatedMotorRps = sotm.motorRps;
                    turretAngleOffsetRad = sotm.turretAngleOffsetRad;
                    double predictedEntryDeg = computeEntryAngleDeg(
                        distance,
                        motorRpsToLaunchSpeedMps(ikSolution.motorRps),
                        Math.toRadians(ikSolution.hoodDegrees)
                    );
                    SmartDashboard.putBoolean("Turret/IK/HasSolution", true);
                    SmartDashboard.putNumber("Turret/IK/RequiredHoodDeg", ikSolution.hoodDegrees);
                    SmartDashboard.putNumber("Turret/IK/RequiredMotorRps", ikSolution.motorRps);
                    SmartDashboard.putNumber("Turret/IK/RequiredCompMotorRps", ikCompensatedMotorRps);
                    SmartDashboard.putNumber("Turret/IK/PredictedEntryDeg", predictedEntryDeg);
                } else {
                    hoodWheelsZero();
                    SmartDashboard.putBoolean("Turret/IK/HasSolution", false);
                    SmartDashboard.putNumber("Turret/IK/RequiredHoodDeg", Double.NaN);
                    SmartDashboard.putNumber("Turret/IK/RequiredMotorRps", Double.NaN);
                    SmartDashboard.putNumber("Turret/IK/RequiredCompMotorRps", Double.NaN);
                    SmartDashboard.putNumber("Turret/IK/test", Double.NaN);
                }
                // TODO
                // Aim the turret at the lead position offset by the SOTM angle correction.
                // turretAngleOffsetRad is 0 when stationary or SOTM is disabled.
                aimTurret(normalizeRadians(errorDegrees + turretAngleOffsetRad - turretPose.getRotation().getRadians()));
                if (shoot) {
                    if (ikSolution != null) {
                        hoodPose.Position = hoodDegreesToRotations(ikSolution.hoodDegrees);
                        velocity = applyShooterControl(ikCompensatedMotorRps);
                    } else {
                        hoodWheelsZero();
                    }
                } else {
                    velocity = aimOnFly(Double.MAX_VALUE, currPose.getRotation().getRadians(), speeds);
                }
            } else {
                aimTurret(normalizeRadians(errorDegrees - turretPose.getRotation().getRadians()));
                velocity = aimOnFly(shoot ? distance : Double.MAX_VALUE, currPose.getRotation().getRadians(), speeds);
            }
        } else {
            hoodWheelsZero();
            SmartDashboard.putNumber("Turret/DistanceToTarget", 0.0);
            SmartDashboard.putBoolean("Turret/IK/HasSolution", false);
            SmartDashboard.putNumber("Turret/IK/RequiredHoodDeg", Double.NaN);
            SmartDashboard.putNumber("Turret/IK/RequiredMotorRps", Double.NaN);
            SmartDashboard.putNumber("Turret/IK/RequiredCompMotorRps", Double.NaN);
            SmartDashboard.putNumber("Turret/IK/PredictedEntryDeg", Double.NaN);
            SmartDashboard.putBoolean("Turret/IK/UsingEntryBand", false);
        }

        boolean mapTuneEnabled = SmartDashboard.getBoolean(
            MapTuneConstants.enableKey,
            MapTuneConstants.defaultEnable
        );
        if (mapTuneEnabled) {
            applyLiveMap();
        }

        if (manualOverride.get()) {
            hoodPose.Position = 0;
            velocity = 0;
            mode = ShootMode.COAST;
            brake = true;
        }

        if (brake) {
            spinMotor.setControl(new StaticBrake());
        } else {
            spinMotor.setControl(spinPose);
        }

        SmartDashboard.putNumber("Turret/SpinAmps", spinMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Turret/SpinSupply", spinMotor.getSupplyCurrent().getValueAsDouble());
        hoodMotor1.setControl(hoodPose);
        SmartDashboard.putNumber("Turret/HoodCurrentDeg", hoodRotationsToDegrees(hoodMotor1.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Turret/ShooterCurrentRps", shootMotor1.getVelocity().getValueAsDouble());
        switch (mode) {
            case DUTY_CYCLE_BANG_BANG -> shootMotor1.setControl(shootDutyBang.withVelocity(velocity));
            case TORQUE_CURRENT_BANG_BANG -> shootMotor1.setControl(shootTorqueBang.withVelocity(velocity));
            case COAST -> shootMotor1.set(0);
        }

        SmartDashboard.putData("Turret Field", m_field);
        SmartDashboard.putNumber("Turret/debug/speeds_field_vx", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Turret/debug/speeds_field_vy", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Turret/debug/robotSpeeds_vx", robotSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Turret/debug/robotSpeeds_vy", robotSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Turret/debug/shooterFOA_deg", Math.toDegrees(currPose.getRotation().getRadians() + turretAngle));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return sysIdRoutineToApply.quasistatic(direction);
    // }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
       * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return sysIdRoutineToApply.dynamic(direction);
    // }
}
