package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;


public class Intake {
    private TalonFX intakeMotorRoller1;
    private TalonFX intakeMotorRoller2;
    private TalonFX intakeMotorDeploy;

    private int intakeID = 0;
    

    public Intake() {
        intakeMotorRoller1 = new TalonFX(intakeID, "intakeMotorRoller1  ");
        intakeMotorRoller2 = new TalonFX(intakeID, "intakeMotorRoller2");
        intakeMotorDeploy = new TalonFX(intakeID, "intakeMotorDeploy");

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

