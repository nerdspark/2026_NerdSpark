package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import java.util.function.Supplier;

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
    private TalonFX leftIntakeRollerMotor;
    private TalonFX rightIntakeRollerMotor;
    private TalonFX intakeMotorDeploy;

     private final TalonFXSimState intakeSim;

    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);
        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration();

      private MotionMagicConfigs motionMagicfastConfigs = intakeDeployMotorConfig.MotionMagic;
      private MotionMagicConfigs motionMagicSlowConfigs = intakeDeployMotorConfig.MotionMagic;


    
    public Intake() {
        intakeMotorDeploy = new TalonFX(IntakeConstants.deployIntakeMotorId,  IntakeConstants.CANBus);
        intakeSim = intakeMotorDeploy.getSimState();
        leftIntakeRollerMotor = new TalonFX(IntakeConstants.leftIntakeMotorRollerId, IntakeConstants.CANBus);
        rightIntakeRollerMotor = new TalonFX(IntakeConstants.rightIntakeMotorRollerId,  IntakeConstants.CANBus);

        intakeDeployMotorConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(IntakeConstants.intakeCurrentLimit)
        .withStatorCurrentLimitEnable(true);
        intakeDeployMotorConfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(0)
        .withSensorToMechanismRatio(1);
        // set slot 0 gains
        intakeDeployMotorConfig.Slot0 = new Slot0Configs()
            .withKP(IntakeConstants.kP)
            .withKI(IntakeConstants.kI)
            .withKD(IntakeConstants.kD)
            .withKG(IntakeConstants.kG)
            .withKS(IntakeConstants.kS)
            .withKA(IntakeConstants.kA)
            .withKV(IntakeConstants.kV);

        

        motionMagicfastConfigs.MotionMagicCruiseVelocity = IntakeConstants.motionMagicCruiseVelocityFast;
        motionMagicfastConfigs.MotionMagicAcceleration = IntakeConstants.motionMagicAcceleration;
        motionMagicfastConfigs.MotionMagicJerk = IntakeConstants.motionMagicJerk;



        motionMagicSlowConfigs.MotionMagicCruiseVelocity = IntakeConstants.motionMagicCruiseVelocitySlow;
        motionMagicSlowConfigs.MotionMagicAcceleration = IntakeConstants.motionMagicAcceleration;
        motionMagicSlowConfigs.MotionMagicJerk = IntakeConstants.motionMagicJerk;

        intakeMotorDeploy
        .getConfigurator()
        .apply(intakeDeployMotorConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)));

        intakeMotorDeploy.getConfigurator().apply(intakeDeployMotorConfig);


    }
    
    public void setDeployPosition(Supplier<Double> rotations) {
        intakeMotorDeploy.setControl(m_mmRequest.withPosition(rotations.get().doubleValue()));
    }
    public void setDeployPower(Supplier<Double> deployPower) {
        intakeMotorDeploy.set(deployPower.get().doubleValue());
    }
    public void setRollerPower(Supplier<Double> rollerPower) {
        leftIntakeRollerMotor.set(-rollerPower.get().doubleValue());
        rightIntakeRollerMotor.set(rollerPower.get().doubleValue());
    }
    public void useFastConfig() {
        intakeMotorDeploy.getConfigurator().apply(motionMagicfastConfigs);
    }
    public void useSlowConfig() {
        intakeMotorDeploy.getConfigurator().apply(motionMagicSlowConfigs);
    }
    public void simulationPeriodic() {
    // double dt = 0.02;

    // Read the applied motor voltage
    double intakeVoltage = intakeSim.getMotorVoltage();

    intakeSim.addRotorPosition(intakeVoltage);
  }
    public void stopIntake() {
            intakeMotorDeploy.stopMotor();
            leftIntakeRollerMotor.stopMotor();
            rightIntakeRollerMotor.stopMotor();


    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Position", intakeMotorDeploy.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Current",intakeMotorDeploy.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Roller Current", leftIntakeRollerMotor.getStatorCurrent().getValueAsDouble());
    }

}

