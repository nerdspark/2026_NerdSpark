package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Intake extends SubsystemBase {
    private CANBus canivore;
    private TalonFX roller1, roller2, deployMotor;

    private final TalonFXSimState intakeSim;

    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);
    private TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();


    private MotionMagicConfigs motionMagicFastConfigs = deployMotorConfig.MotionMagic;
    private MotionMagicConfigs motionMagicSlowConfigs = deployMotorConfig.MotionMagic;

    public Intake() {
        canivore = new CANBus(Constants.CANbus);
        deployMotor = new TalonFX(IntakeConstants.deployIntakeMotorId,  canivore);
        intakeSim = deployMotor.getSimState();
        roller1 = new TalonFX(IntakeConstants.roller1id, canivore);
        roller2 = new TalonFX(IntakeConstants.roller2id,  canivore);

        rollerMotorConfig.MotorOutput = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        rollerMotorConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeConstants.rollerStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeConstants.rollerSupplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true);

        deployMotorConfig.MotorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        deployMotorConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeConstants.deployStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeConstants.deploySupplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true);
        deployMotorConfig.Feedback = new FeedbackConfigs()
            .withFeedbackRotorOffset(0)
            .withSensorToMechanismRatio(1);
        deployMotorConfig.Slot0 = new Slot0Configs()
            .withKP(IntakeConstants.kP)
            .withKI(IntakeConstants.kI)
            .withKD(IntakeConstants.kD)
            .withKG(IntakeConstants.kG)
            .withKS(IntakeConstants.kS)
            .withKA(IntakeConstants.kA)
            .withKV(IntakeConstants.kV);

        motionMagicFastConfigs.MotionMagicCruiseVelocity = IntakeConstants.motionMagicCruiseVelocityFast;
        motionMagicFastConfigs.MotionMagicAcceleration = IntakeConstants.motionMagicAcceleration;
        motionMagicFastConfigs.MotionMagicJerk = IntakeConstants.motionMagicJerk;

        motionMagicSlowConfigs.MotionMagicCruiseVelocity = IntakeConstants.motionMagicCruiseVelocitySlow;

        deployMotor.getConfigurator().apply(deployMotorConfig);
        roller1.getConfigurator().apply(rollerMotorConfig);
        roller2.getConfigurator().apply(rollerMotorConfig);
    }

    public Boolean intakeIsIn() {
        return deployMotor.getPosition().getValueAsDouble() < 3;
    }

    public double getDeployPosition() {
        return deployMotor.getPosition().getValueAsDouble();
    }
    
    public void setDeployPosition(Supplier<Double> rotations) {
        deployMotor.setControl(m_mmRequest.withPosition(rotations.get().doubleValue()));
    }
    
    public void setRollerPower(double power) {
        roller1.set(-power);
        roller2.set(power);
    }

    public boolean rollerOn() {
        return Math.abs(roller1.get()) + Math.abs(roller2.get()) > 0.1;
    }

    public void useFastConfig() {
        deployMotor.getConfigurator().apply(motionMagicFastConfigs);
    }
    public void useSlowConfig() {
        deployMotor.getConfigurator().apply(motionMagicSlowConfigs);
    }

    public void simulationPeriodic() {
        // double dt = 0.02;
        // Read the applied motor voltage
        double intakeVoltage = intakeSim.getMotorVoltage();
        intakeSim.addRotorPosition(intakeVoltage);
    }

    public void stopIntake() {
        deployMotor.stopMotor();
        roller1.stopMotor();
        roller2.stopMotor();
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Position", deployMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Current", deployMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Roller Current", roller1.getStatorCurrent().getValueAsDouble());
    }

}

