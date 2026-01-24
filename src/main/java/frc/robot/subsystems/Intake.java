package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;


public class Intake implements Subsystem{
    private TalonFX intakeMotorRoller1;
    private TalonFX intakeMotorRoller2;
    private TalonFX intakeMotorDeploy;

    private int intakeMotorRoller1ID;
    private int intakeMotorRoller2ID;
    private int intakeMotorDeployID;
    

    public Intake() {
        intakeMotorRoller1 = new TalonFX(intakeMotorRoller1ID, "intakeMotorRoller1  ");
        intakeMotorRoller2 = new TalonFX(intakeMotorRoller2ID, "intakeMotorRoller2");
        intakeMotorDeploy = new TalonFX(intakeMotorDeployID, "intakeMotorDeploy");

        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration intakeMotorRoller1 = new TalonFXConfiguration();
        TalonFXConfiguration intakeMotorRoller2 = new TalonFXConfiguration();
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
    public void setDeployPosition(double position) {
        intakeMotorDeploy.setPosition(position);
    }
}

