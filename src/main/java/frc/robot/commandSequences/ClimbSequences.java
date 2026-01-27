package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

//Start of Climb Sequence
//L1-------------------------
//Taller Arm goes to 30 inches and hooks on first rung and Small Arm goes to highest position
//Tall Arm pulls down until small arm then small arm hooks on first rung and Tall Arm lets go
//L2-------------------------
//Tall Arm goes to second rung position and hooks on second rung
//Small Arm unhooks from first rung
//Kicker Arm goes out and pushes robot back and allows clearance
//Tall Arm pulls down until small arm then small arm hooks on second rung and Tall Arm lets go
//L3-------------------------
//Tall Arm goes to third rung position and hooks on third rung
//Small Arm unhooks from second rung
//Kicker Arm goes out and pushes robot back and allow clearance
//Tall Arm pulls down until small arm then small arm hooks on third rung and Tall Arm lets go.
//End Climb---------------------


public class ClimbSequences {
    public static Command climbToL1(Climb climb) {

        //L1-------------------------
        //Taller Arm goes to 30 inches and hooks on first rung and Small Arm goes to highest position
        //Tall Arm pulls down until small arm then small arm hooks on first rung and Tall Arm lets go
        return new SequentialCommandGroup(
            climb.tallGoToPosition(() -> ClimbConstants.l1Position),
            new WaitCommand(0.5),
            climb.tallResetPosition()
        );
    }

    public static Command climbToL2(Climb climb) {
        //Tall Arm goes to second rung position and hooks on second rung
        //Small Arm unhooks from first rung
        //Kicker Arm goes out and pushes robot back and allows clearance
        //Tall Arm pulls down until small arm then small arm hooks on second rung and Tall Arm lets go
        return new SequentialCommandGroup(

        );
    }

    //Tall Arm goes to third rung position and hooks on third rung
    //Small Arm unhooks from second rung
    //Kicker Arm goes out and pushes robot back and allow clearance
    //Tall Arm pulls down until small arm then small arm hooks on third rung and Tall Arm lets go.

    public static Command climbToL3(Climb climb) {
        return new SequentialCommandGroup(

        );
    }
}