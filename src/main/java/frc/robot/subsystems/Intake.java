package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private double deployPositionRot;
    private double rollerPower;

    public Intake() {
        deployPositionRot = 0.0;
        rollerPower = 0.0;
    }

    public void setDeployPosition(Supplier<Double> rotations) {
        deployPositionRot = rotations.get();
    }

    public void setRollerPower(double power) {
        rollerPower = power;
    }

    public void stopIntake() {
        rollerPower = 0.0;
    }

    public double getDeployPositionRot() {
        return deployPositionRot;
    }

    public double getRollerPower() {
        return rollerPower;
    }
}
