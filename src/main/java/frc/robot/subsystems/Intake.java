package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotorRoller1;
    private TalonFX intakeMotorRoller2;
    private TalonFX intakeMotorDeploy;

    private int intakeMotorRoller1ID = 1;
    private int intakeMotorRoller2ID = 2;
    private int intakeMotorDeployID = 3;
    
    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);
    
    public Intake() {
        intakeMotorRoller1 = new TalonFX(intakeMotorRoller1ID, "intakeMotorRoller1");
        intakeMotorRoller2 = new TalonFX(intakeMotorRoller2ID, "intakeMotorRoller2");
        intakeMotorDeploy = new TalonFX(intakeMotorDeployID, "intakeMotorDeploy");

        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration();
        // set slot 0 gains
        var slot0Configs = intakeDeployMotorConfig.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = intakeDeployMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        intakeMotorDeploy.getConfigurator().apply(intakeDeployMotorConfig);


    }
    
    public void setDeployPosition(double rotations) {
        intakeMotorDeploy.setControl(intakeMotorDeploy);
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

