package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Intake extends SubsystemBase {
    private TalonFX leftIntakeRollerMotor;
    private TalonFX rightIntakeRollerMotor;
    private TalonFX intakeMotorDeploy;

    
    
    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);
        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration();

      private MotionMagicConfigs motionMagicConfigs = climbConfig.MotionMagic;

    
    public Intake() {
        leftIntakeRollerMotor = new TalonFX(IntakeConstants.leftIntakeMotorRollerId, IntakeConstants.CANBus);
        rightIntakeRollerMotor = new TalonFX(IntakeConstants.rightIntakeMotorRollerId,  IntakeConstants.CANBus);
        intakeMotorDeploy = new TalonFX(IntakeConstants.deployIntakeMotorId,  IntakeConstants.CANBus);

        intakeDeployMotorConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(IntakeConstants.intakeCurrentLimit)
        .withStatorCurrentLimitEnable(true);
        intakeDeployMotorConfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(0)
        .withSensorToMechanismRatio(1);
        // set slot 0 gains
        intakeDeployMotorConfig.Slo0 = new Slot0Configs()
            .withKP(IntakeConstants.kP)
            .withKI(IntakeConstants.kI)
            .withKD(IntakeConstants.kD)
            .withKG(IntakeConstants.kG)
            .withKS(IntakeConstants.kS)
            .withKA(IntakeConstants.kA)
            .withKV(IntakeConstants.kV)

        

        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.motionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.motionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = IntakeConstants.motionMagicJerk;

        intakeMotorDeploy
        .getConfigurator()
        .apply(intakeDeployMotorConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)));

        intakeMotorDeploy.getConfigurator().apply(intakeDeployMotorConfig);


    }
    
    public void setDeployPosition(double rotations) {
        intakeMotorDeploy.setControl(m_mmRequest.withPosition(rotations));
    }
    
    public void setRollerPower(double power) {
        intakeMotorRoller1.set(power);
        intakeMotorRoller2.set(power);
    }
    public void stopIntake() {
            intakeMotorDeploy.stopMotor();
            intakeMotorRoller1.stopMotor();
            intakeMotorRoller2.stopMotor();

    }

}

