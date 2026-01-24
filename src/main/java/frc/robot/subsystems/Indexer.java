package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.field;
import frc.robot.Constants.turretConstants;

public class Indexer implements Subsystem {

    private TalonFX conveyorMotor, passThroughMotor, rightDrumMotor, leftDrumMotor;
    

    public Indexer() {
        // pose = robotPose;
        // alliance = driverAlliance;
        // this.aimTurret = aimTurret;

        conveyorMotor = new TalonFX(0); //TODO add these in Later
        passThroughMotor = new TalonFX(1); // TODO add these in later
        
        // drum motors:
        rightDrumMotor = new TalonFX(2); // TODO add these later
        leftDrumMotor = new TalonFX(3); // TODO add these later

        // indexer configs
        TalonFXConfiguration conveyorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;

        TalonFXConfiguration passThroughConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;

        // drum motor configs: 
        TalonFXConfiguration rightDrumMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;

        TalonFXConfiguration leftDrumMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;

        conveyorMotor.getConfigurator().apply(conveyorConfig);
        passThroughMotor.getConfigurator().apply(passThroughConfig);

        //set drum motor configs
        rightDrumMotor.getConfigurator().apply(rightDrumMotorConfig);
        leftDrumMotor.getConfigurator().apply(leftDrumMotorConfig);
        
    }
    
    // Moves the the pass through motors and spins the conveyer belt.
    public void passThrough(Supplier<Boolean> isActive) {
        if(isActive.get()){
            conveyorMotor.set(0.2);
            passThroughMotor.set(0.2);
        } else {
            conveyorMotor.set(0.0);
            passThroughMotor.set(0.0);
        }
        
    }

    // control the drum motors
    public void moveDrumMotors(Supplier<Boolean> isActive) {
        if(isActive.get()){
            rightDrumMotor.set(0.2);
            leftDrumMotor.set(-0.2); // TODO might need to be swapped
        } else {
            rightDrumMotor.set(0.0);
            leftDrumMotor.set(0.0);
        }
    }

    @Override
    public void periodic() {
        
    }

    
}
