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

    private TalonFX conveyorMotor, passThroughMotor;
    

    public Indexer(Supplier<Pose2d> robotPose, Supplier<DriverStation.Alliance> driverAlliance, Supplier<Boolean> aimTurret) {
        // pose = robotPose;
        // alliance = driverAlliance;
        // this.aimTurret = aimTurret;

        conveyorMotor = new TalonFX(0);
        passThroughMotor = new TalonFX(0);
        

        TalonFXConfiguration conveyorConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration passThroughConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;
        

        conveyorMotor.getConfigurator().apply(conveyorConfig);
        passThroughMotor.getConfigurator().apply(passThroughConfig);
        
    }
    
    // Moves the the pass through motors and spins the conveyer belt.
    public void passThrough() {
        conveyorMotor.set(0.2);
        passThroughMotor.set(0.2);
    }


    @Override
    public void periodic() {
        
    }

    
}
