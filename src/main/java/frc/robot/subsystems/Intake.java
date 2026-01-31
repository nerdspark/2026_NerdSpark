package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Intake extends SubsystemBase {
    private CANBus canivore;
    private TalonFX intakeMotorRoller1, intakeMotorRoller2, intakeMotorDeploy;
    
    private final MotionMagicVoltage deploy = new MotionMagicVoltage(0);
    
    public Intake() {
        canivore = new CANBus(Constants.CANbus);

        intakeMotorRoller1 = new TalonFX(IntakeConfig.intakeMotorRoller1ID, canivore);
        intakeMotorRoller2 = new TalonFX(IntakeConfig.intakeMotorRoller2ID, canivore);
        intakeMotorDeploy = new TalonFX(IntakeConfig.intakeMotorDeployID, canivore);

        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration().withSlot0(
            new Slot0Configs()
                .withKP(IntakeConfig.Kp)
                .withKI(IntakeConfig.Ki)
                .withKD(IntakeConfig.Kd)
                .withKS(IntakeConfig.Ks)
                .withKV(IntakeConfig.Kv)
                .withKA(IntakeConfig.Ka)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(IntakeConfig.velo)
            .withMotionMagicAcceleration(IntakeConfig.accel)
            .withMotionMagicJerk(IntakeConfig.jerk)
        );

        intakeMotorDeploy.getConfigurator().apply(intakeDeployMotorConfig);
    }
    
    public void setDeployPosition(double rotations) {
        deploy.Position = rotations;
        intakeMotorDeploy.setControl(deploy);
    }
    
    public void setDeployPower(double target){
        intakeMotorDeploy.set(target);
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