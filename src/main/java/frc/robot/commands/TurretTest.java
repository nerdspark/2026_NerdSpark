package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretTest extends Command {
    private Turret turret;
    private double speed;

    public TurretTest(Turret turret, double speed) {
        this.turret = turret;
        this.speed = speed;

        addRequirements(turret);
    }

    public void execute() {
        turret.calcBangBang(speed);
    }

    public void end(boolean interrupted) {
        turret.calcBangBang(0);
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    } 
}