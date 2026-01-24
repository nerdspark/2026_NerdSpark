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
import frc.robot.Constants.indexerConstants;
import frc.robot.Constants.turretConstants;

public class Indexer implements Subsystem {

    private TalonFX conveyorMotor, DrumMotor;
    

    public Indexer() {

        conveyorMotor = new TalonFX(0); //TODO add these in Later
        DrumMotor = new TalonFX(1); // TODO add these later
        
        // indexer configs
        TalonFXConfiguration conveyorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;

        // drum motor configs: 
        TalonFXConfiguration DrumMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;

        

        conveyorMotor.getConfigurator().apply(conveyorConfig);

        //set drum motor configs
        DrumMotor.getConfigurator().apply(DrumMotorConfig);
        
        
    }
    
    // Moves the the pass through motors and spins the conveyer belt.
    public void passThrough(Supplier<Boolean> isActive, Supplier<Double> rollerSpeed) {
        if(isActive.get()){
            conveyorMotor.set(rollerSpeed.get());
        } else {
            conveyorMotor.set(0.0);
        }
        
    }

    // control the drum motors
    public void moveDrumMotors(Supplier<Boolean> isActive) {
        if(isActive.get()){
            DrumMotor.set(indexerConstants.DRUM_MOTOR_SPEED);
        } else {
            DrumMotor.set(0.0);
        }
    }

    @Override
    public void periodic() {
        
    }

    
}
