package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.TurretUtil.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.util.ShooterOffsetMap;

public class Turret extends SubsystemBase {
    private static final double TWO_PI = 2.0 * Math.PI;

    public double tof = 0;
    
    private CANBus canivore;
    private TalonFX spinMotor, hoodMotor1, hoodMotor2, shootMotor1, shootMotor2;
    private CANcoder spinCancoder1, spinCancoder2;

    private VelocityDutyCycle shootDutyBang = new VelocityDutyCycle(0);
    private VelocityTorqueCurrentFOC shootTorqueBang = new VelocityTorqueCurrentFOC(0);
    private MotionMagicVoltage hoodPose = new MotionMagicVoltage(0);
    private MotionMagicVoltage spinPose = new MotionMagicVoltage(0);

    // private VoltageOut sysId = new VoltageOut(0);

    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speed;
    private Supplier<Boolean> climb;

    private boolean shortPathCrossesWrap;
    private boolean pathLatched = false;
    private double turretAngle;

    private double filteredTurretDelaySec = TurretConstants.delay;
    public static double delaySum = 0.0;
    public static int delaySamples = 0;
    public static double maxDelay = 0.0;


    private Debouncer torqueCurrentDebouncer = new Debouncer(0.02, DebounceType.kFalling);
    private ShootMode mode = ShootMode.COAST;
    private double velocity = 0;

    private final Field2d m_field = new Field2d();
    private final ShooterOffsetMap offsetMap = new ShooterOffsetMap();

    public Turret(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds, Supplier<Boolean> climbing) {
        pose = robotPose;
        speed = speeds;
        climb = climbing;

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
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(TurretConfig.spinVelocity)
                .withMotionMagicAcceleration(TurretConfig.spinAccel)
            )
        ;
        TalonFXConfiguration hoodConfig1 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
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
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(TurretConfig.hoodVelocity)
                .withMotionMagicAcceleration(TurretConfig.hoodAccel)
            )
        ;
        TalonFXConfiguration shootConfig1 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
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
     * Calculates the dynamic X threshold line for a single state (shoot or pass),
     * taking into account velocity and direction.
     *
     * @param startingLine      Original X coordinate of the line
     * @param robotX            Current robot X position
     * @param robotVeloX        Current robot X velocity
     * @param safetyMargin      Extra distance to start retracting early
     * @param approachDirection The sign of the robot velo going toward the line
     * @return X value of dynamic trigger line
     */
    private double calcTriggerLine(double startingLine, double robotX, double robotVeloX, double safetyMargin, int approachDirection) {
        // Closing velocity toward line
        double closingVelocity = robotVeloX * approachDirection;

        // Only extend the line if moving toward it
        if (closingVelocity <= 0) {
            // Moving away or stopped, use original line
            return startingLine + safetyMargin;
        }

        // Distance traveled while retracting
        double preTriggerDistance = (closingVelocity * TurretConstants.hoodRetractTime) + safetyMargin;

        // Move the line backward along approach direction
        return startingLine - (preTriggerDistance * approachDirection);
    }

    /** 
     * Aims the hood of the turret and spins wheels based on shooter map and chassis speeds
     * If hood is not tight then look into chassis velocity based correction for hood
     * 
     * @param distance the distance to the center of the hub from the center of the robot
     * @return the velocity to shoot at
    */
    private double aimOnFly(double distance) {
        ShooterParams map;
        if (climb.get()) {
            map = TurretConstants.climbMap.get(distance);
        } else {
            map = TurretConstants.map.get(distance);
        }

        hoodPose.Position = map.hoodPose;

        double robotHeading = pose.get().getRotation().getRadians();
        double shooterFOA = robotHeading + turretAngle;
        ChassisSpeeds robotFOS = speed.get();
        double robotSpeed = Math.hypot(robotFOS.vxMetersPerSecond, robotFOS.vyMetersPerSecond);
        double robotVelAngle = Math.atan2(robotFOS.vyMetersPerSecond, robotFOS.vxMetersPerSecond);
        double vParallel = robotSpeed * Math.cos(robotVelAngle - shooterFOA);
        double deltaMotorRPS = vParallel / (2.0 * Math.PI * TurretConstants.shooterWheelRadius);

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
        double theta1 = spinCancoder1.getAbsolutePosition().getValueAsDouble() * TWO_PI;
        double theta2 = spinCancoder2.getAbsolutePosition().getValueAsDouble() * TWO_PI;
        
        turretAngle = floorMod(
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
        turretAngle = normalizeRadians(turretAngle);
        SmartDashboard.putNumber("Turret Angle", Math.toDegrees(turretAngle));

        neededAngle = normalizeRadians(neededAngle + 180);
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(neededAngle));

        // Update phase delay here, 20ms loop
        estimateTurretPhaseDelaySec(
            neededAngle, 
            turretAngle, 
            spinMotor.getVelocity().getValueAsDouble() * TWO_PI, 
            0.02
        );

        // Compute angular error
        double error = neededAngle - turretAngle;

        // Find the shortest path
        double shortError = error;
        if (shortError > Math.PI) {
            shortError -= TWO_PI;
        } else if (shortError < -Math.PI) {
            shortError += TWO_PI;
        }

        if (!pathLatched) {
            // Does the short path cross the wrap?
            shortPathCrossesWrap =
                Math.abs(turretAngle) > Math.PI / 2.0 &&
                Math.abs(neededAngle) > Math.PI / 2.0 &&
                Math.signum(turretAngle) != Math.signum(neededAngle);

            pathLatched = true;
        }

        // Select legal error
        double chosenError = shortPathCrossesWrap ? error : shortError;

        if (pathLatched && Math.abs(chosenError) < Math.toRadians(90)) {
            pathLatched = false;
        }

        // Command motor
        double motorDelta = (chosenError / TWO_PI) * TurretConstants.spinRatio;
        spinPose.Position = motorDelta + spinMotor.getPosition().getValueAsDouble();
    }

    private double applyShooterControl(double motorRps) {
        if (motorRps < 5) {
            mode = ShootMode.COAST;
            return 0;
        }

        boolean inTolerance = Math.abs(shootMotor1.getVelocity().getValueAsDouble() - motorRps) <= 3;
        boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
        mode = torqueCurrentControl ? ShootMode.TORQUE_CURRENT_BANG_BANG : ShootMode.DUTY_CYCLE_BANG_BANG;

        return motorRps;
    }

    private IkSolution solveIK(double distanceMeters) {
        if (distanceMeters <= 0.0) {
            return null;
        }
        double deltaHeight = TurretConstants.targetHeightMeters - TurretConstants.shooterMuzzleHeightMeters;
        double alpha = Math.atan2(deltaHeight, distanceMeters);
        double thetaOptimal = 0.5 * (alpha + (Math.PI / 2.0));

        double minAngle = Math.toRadians(TurretConstants.hoodMinDegrees);
        double maxAngle = Math.toRadians(TurretConstants.hoodMaxDegrees);
        double thetaClamped = Math.max(minAngle, Math.min(maxAngle, thetaOptimal));

        double bestTheta = Double.NaN;
        double bestMotorRps = Double.POSITIVE_INFINITY;
        double[] candidates = {thetaClamped, minAngle, maxAngle};
        for (double theta : candidates) {
            double speedMps = solveIKSpeed(distanceMeters, alpha, theta);
            if (!Double.isFinite(speedMps) || speedMps <= 0.0) {
                continue;
            }
            double wheelRps = speedMps / (2.0 * Math.PI * TurretConstants.shooterWheelRadius);
            double motorRps = wheelRps * TurretConstants.shooterRatio;
            if (motorRps <= TurretConstants.shooterMaxMotorRps && motorRps < bestMotorRps) {
                bestMotorRps = motorRps;
                bestTheta = theta;
            }
        }

        if (!Double.isFinite(bestTheta)) {
            return null;
        }

        ShooterOffsetMap.Offsets offsets = offsetMap.sample(distanceMeters);
        double hoodDeg = Math.max(
            TurretConstants.hoodMinDegrees,
            Math.min(
                TurretConstants.hoodMaxDegrees,
                Math.toDegrees(bestTheta) + offsets.hoodOffsetDeg
            )
        );
        double motorRps = Math.max(
            0.0,
            Math.min(TurretConstants.shooterMaxMotorRps, bestMotorRps + offsets.motorRpsOffset)
        );
        return new IkSolution(hoodDeg, motorRps);
    }

    private double solveIKSpeed(double distanceMeters, double alpha, double theta) {
        double denominator = 2.0 * Math.cos(theta) * Math.sin(theta - alpha);
        if (denominator <= 1e-9) {
            return Double.NaN;
        }
        double vSquared = (9.80665 * distanceMeters * Math.cos(alpha)) / denominator;
        if (vSquared <= 0.0) {
            return Double.NaN;
        }
        return Math.sqrt(vSquared);
    }

    private double applyChassisVelocityComp(double motorRps) {
        double robotHeading = pose.get().getRotation().getRadians();
        double shooterFOA = robotHeading + turretAngle;
        ChassisSpeeds robotFOS = speed.get();
        double robotSpeed = Math.hypot(robotFOS.vxMetersPerSecond, robotFOS.vyMetersPerSecond);
        double robotVelAngle = Math.atan2(robotFOS.vyMetersPerSecond, robotFOS.vxMetersPerSecond);
        double vParallel = robotSpeed * Math.cos(robotVelAngle - shooterFOA);
        double deltaMotorRPS = vParallel / (2.0 * Math.PI * TurretConstants.shooterWheelRadius);
        return motorRps - deltaMotorRPS;
    }

    private double hoodDegreesToRotations(double hoodDegrees) {
        return (hoodDegrees / 360.0) * TurretConstants.hoodRatio;
    }

    @Override
    public void periodic() {
        ChassisSpeeds speeds = speed.get();
        Pose2d currPose = pose.get();
        m_field.setRobotPose(currPose);
        Pose2d delayPose = currPose.exp(new Twist2d( // Account for phase delay
            speeds.vxMetersPerSecond * filteredTurretDelaySec, 
            speeds.vyMetersPerSecond * filteredTurretDelaySec,
            speeds.omegaRadiansPerSecond * filteredTurretDelaySec
        ));
        SmartDashboard.putNumber("Phase Delay", filteredTurretDelaySec);
        m_field.getObject("Delay Pose").setPose(delayPose);
        Translation2d rotationOffset = TurretConstants.robotToTurret.rotateBy(delayPose.getRotation());
        Pose2d turretPose = new Pose2d(delayPose.getTranslation().plus(rotationOffset), delayPose.getRotation());

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
        SmartDashboard.putBoolean("Is Blue", isBlue);
        double shootLine = calcTriggerLine(
            isBlue ? FieldConstants.LinesVertical.blueShootLine : FieldConstants.LinesVertical.redShootLine, 
            delayPose.getX(), 
            speeds.vxMetersPerSecond, 
            0.25, 
            isBlue ? 1 : -1
        );
        double passLine = calcTriggerLine(
            isBlue ? FieldConstants.LinesVertical.bluePassLine : FieldConstants.LinesVertical.redPassLine, 
            delayPose.getX(), 
            speeds.vxMetersPerSecond, 
            0.25, 
            isBlue ? -1 : 1
        );

        boolean shoot = isBlue ? turretPose.getX() <= shootLine : turretPose.getX() >= passLine;
        SmartDashboard.putBoolean("Shoot", shoot);
        boolean pass = isBlue ? turretPose.getX() >= shootLine : turretPose.getX() <= passLine; 
        SmartDashboard.putBoolean("Pass", pass);

        if (shoot || pass) {
            delaySum += filteredTurretDelaySec;
            delaySamples++;
            maxDelay = Math.max(maxDelay, filteredTurretDelaySec);

            boolean forceTarget = SmartDashboard.getBoolean(
                turretTargetConstants.enableKey,
                turretTargetConstants.defaultEnable
            );

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
                    passPose = closerPoint(turretPose, FieldConstants.LeftBump.oppNearLeftCorner, FieldConstants.RightBump.oppNearLeftCorner) 
                        ? FieldConstants.LeftBump.oppNearLeftCorner : FieldConstants.RightBump.oppNearLeftCorner;
                }
            }

            Translation2d targetPose = shoot ? goalPose : passPose;
            m_field.getObject("Target Pose").setPose(targetPose.getMeasureX(), targetPose.getMeasureY(), new Rotation2d());
            double distance = turretPose.getTranslation().getDistance(targetPose);

            tof = TurretConstants.map.get(distance).tof; // Lookup TOF from table
            Translation2d lookaheadTurretPos = turretPose.getTranslation();

            for (int i = 0; i < 10; i++) {
                Translation2d robotFieldVelocity = new Translation2d(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond
                );
                Translation2d flightOffset = robotFieldVelocity.times(tof); // How far robot moves during ball flight
                lookaheadTurretPos = turretPose.getTranslation().plus(flightOffset); // Effective launch point
                distance = lookaheadTurretPos.getDistance(targetPose); // Recompute distance
                tof = TurretConstants.map.get(distance).tof;       // Recompute TOF for new distance
            }
            m_field.getObject("Look Ahead Pose").setPose(lookaheadTurretPos.getMeasureX(), lookaheadTurretPos.getMeasureY(), new Rotation2d());

            double xError = targetPose.getX() - lookaheadTurretPos.getX();
            double yError = targetPose.getY() - lookaheadTurretPos.getY();
            double errorDegrees = Math.atan2(yError, xError);
                        
            aimTurret(normalizeRadians(errorDegrees - turretPose.getRotation().getRadians()));
            if (forceTarget) {
                boolean useIK = SmartDashboard.getBoolean(
                    AutoAimConstants.useIKSolverKey,
                    AutoAimConstants.defaultUseIKSolver
                );
                if (useIK) {
                    IkSolution solution = solveIK(distance);
                    if (solution != null) {
                        hoodPose.Position = hoodDegreesToRotations(solution.hoodDegrees);
                        double compensated = applyChassisVelocityComp(solution.motorRps);
                        velocity = applyShooterControl(compensated);
                    } else {
                        hoodWheelsZero();
                    }
                } else {
                    velocity = aimOnFly(distance);
                }
            } else {
                velocity = aimOnFly(shoot ? Math.hypot(yError, xError) : Double.MAX_VALUE);
            }
        } else {
            hoodWheelsZero();
        }

        spinMotor.setControl(spinPose);
        hoodMotor1.setControl(hoodPose);
        
        switch (mode) {
            case DUTY_CYCLE_BANG_BANG -> shootMotor1.setControl(shootDutyBang.withVelocity(velocity));
            case TORQUE_CURRENT_BANG_BANG -> shootMotor1.setControl(shootTorqueBang.withVelocity(velocity));
            case COAST -> shootMotor1.set(0);
        }

        SmartDashboard.putData("Turret Field", m_field);
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

    private static final class IkSolution {
        private final double hoodDegrees;
        private final double motorRps;

        private IkSolution(double hoodDegrees, double motorRps) {
            this.hoodDegrees = hoodDegrees;
            this.motorRps = motorRps;
        }
    }
}
