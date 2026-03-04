package frc.robot.commandSequences;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb.LeftClimb;
import frc.robot.subsystems.Climb.RightClimb;

// TODO: Add LED state variable

public class ClimbSequences {

        // public static Command climbMasterReset(LeftClimb leftClimb, RightClimb
        // rightClimb) {
        // return new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // leftClimb.configMotionMagic(ClimbConstants.motionMagicCruiseVelocity,
        // ClimbConstants.motionMagicAcceleration,
        // ClimbConstants.motionMagicJerk);
        // rightClimb.configMotionMagic(ClimbConstants.motionMagicCruiseVelocity,
        // ClimbConstants.motionMagicAcceleration,
        // ClimbConstants.motionMagicJerk);
        // }),
        // new ParallelCommandGroup(
        // leftClimb.leftGoToPosition(() -> 0.0),
        // rightClimb.rightGoToPosition(() -> 0.0)));
        // }

        public static Command controlledDescent(LeftClimb leftClimb, RightClimb rightClimb) {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        leftClimb.configMotionMagic(ClimbConstants.motionMagicDescendCruiseVelocity,
                                                        ClimbConstants.motionMagicDescendAcceleration,
                                                        ClimbConstants.motionMagicDescendJerk);
                                        rightClimb.configMotionMagic(ClimbConstants.motionMagicDescendCruiseVelocity,
                                                        ClimbConstants.motionMagicDescendAcceleration,
                                                        ClimbConstants.motionMagicDescendJerk);
                                }),
                                new ParallelCommandGroup(
                                                leftClimb.leftGoToPosition(() -> ClimbConstants.l1Position),
                                                rightClimb.rightGoToPosition(() -> ClimbConstants.l1Position)));
        }

        public static Command innerHooksUp(LeftClimb leftClimb, RightClimb rightClimb) {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        leftClimb.configMotionMagic(ClimbConstants.motionMagicCruiseVelocity,
                                                        ClimbConstants.motionMagicAcceleration,
                                                        ClimbConstants.motionMagicJerk);
                                        rightClimb.configMotionMagic(ClimbConstants.motionMagicCruiseVelocity,
                                                        ClimbConstants.motionMagicAcceleration,
                                                        ClimbConstants.motionMagicJerk);
                                }),
                                new ParallelCommandGroup(
                                                leftClimb.leftGoToPosition(() -> 0.0),
                                                rightClimb.rightGoToPosition(() -> 0.0)));
        }

        public static Command outerHooksUp(LeftClimb leftClimb, RightClimb rightClimb) {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        leftClimb.configMotionMagic(ClimbConstants.motionMagicCruiseVelocity,
                                                        ClimbConstants.motionMagicAcceleration,
                                                        ClimbConstants.motionMagicJerk);
                                        rightClimb.configMotionMagic(ClimbConstants.motionMagicCruiseVelocity,
                                                        ClimbConstants.motionMagicAcceleration,
                                                        ClimbConstants.motionMagicJerk);
                                }),
                                new ParallelCommandGroup(
                                                leftClimb.leftGoToPosition(() -> ClimbConstants.l1Position),
                                                rightClimb.rightGoToPosition(() -> ClimbConstants.l1Position)));
        }

        public static Command climbTol1(LeftClimb leftClimb, RightClimb rightClimb) {
                return new SequentialCommandGroup(
                        new InstantCommand(() -> {
                                leftClimb.configMotionMagic(ClimbConstants.motionMagicCruiseVelocity,
                                                        ClimbConstants.motionMagicAcceleration,
                                                        ClimbConstants.motionMagicJerk);
                                rightClimb.configMotionMagic(ClimbConstants.motionMagicCruiseVelocity,
                                                        ClimbConstants.motionMagicAcceleration,
                                                        ClimbConstants.motionMagicJerk);
                        }),
                        new ParallelCommandGroup(
                                leftClimb.leftGoToPosition(() -> ClimbConstants.l1Position),
                                rightClimb.rightGoToPosition(() -> ClimbConstants.l1Position)),
                        new WaitCommand(0.3),
                        new ParallelCommandGroup(
                                leftClimb.leftGoToPosition(() -> ClimbConstants.l1PositionWhenClimbed),
                                rightClimb.rightGoToPosition(() -> ClimbConstants.l1PositionWhenClimbed))
                );
        }

        // public static Command climbTol3() {

        // }

        // public static Command climbMasterReset(TallClimb tallClimb, ShortClimb
        // shortClimb) {
        // return new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // tallClimb.configMotionMagic(
        // ClimbConstants.motionMagicCruiseVelocity,
        // ClimbConstants.motionMagicAcceleration,
        // ClimbConstants.motionMagicJerk);
        // shortClimb.configMotionMagic(
        // ClimbConstants.motionMagicCruiseVelocity,
        // ClimbConstants.motionMagicAcceleration,
        // ClimbConstants.motionMagicJerk);
        // }),
        // new ParallelCommandGroup(
        // tallClimb.tallGoToPosition(() -> 0.0),
        // shortClimb.shortGoToPosition(() -> 0.0)));
        // }

        // // public static Command tallArmUp(TallClimb tallClimb, ShortClimb
        // shortClimb) {
        // // return new SequentialCommandGroup(
        // // tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position)
        // // );
        // // }

        // public static Command climbToL1(TallClimb tallClimb, ShortClimb shortClimb) {
        // return new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // tallClimb.configMotionMagic(ClimbConstants.motionMagicCruiseVelocity,
        // ClimbConstants.motionMagicAcceleration,
        // ClimbConstants.motionMagicJerk);
        // shortClimb.configMotionMagic(
        // ClimbConstants.motionMagicCruiseVelocity,
        // ClimbConstants.motionMagicAcceleration,
        // ClimbConstants.motionMagicJerk);
        // }),
        // new ParallelCommandGroup(
        // tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
        // shortClimb.shortGoToPosition(() -> ClimbConstants.l1Position)),
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
        // tallClimb.tallGoToPosition(() -> ClimbConstants.l1PositionWhenClimbed),
        // shortClimb.shortGoToPosition(
        // () -> ClimbConstants.l1PositionWhenClimbed)));
        // }

        // public static Command controlledDescent(TallClimb tallClimb, ShortClimb
        // shortClimb) {
        // return new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // tallClimb.configMotionMagic(ClimbConstants.motionMagicDescendCruiseVelocity,
        // ClimbConstants.motionMagicDescendAcceleration,
        // ClimbConstants.motionMagicDescendJerk);
        // shortClimb.configMotionMagic(
        // ClimbConstants.motionMagicDescendCruiseVelocity,
        // ClimbConstants.motionMagicDescendAcceleration,
        // ClimbConstants.motionMagicDescendJerk);
        // }),
        // new ParallelCommandGroup(
        // tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
        // shortClimb.shortGoToPosition(() -> ClimbConstants.l1Position)));
        // }

        // // public static Command climbToL2(TallClimb tallClimb, ShortClimb
        // shortClimb) {
        // // return new SequentialCommandGroup(
        // // new WaitCommand(0.3),
        // // new ParallelCommandGroup(
        // // tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
        // // shortClimb.shortGoToPosition(() -> 0.0)),
        // // new WaitCommand(0.3),
        // // new ParallelCommandGroup(
        // // tallClimb.tallGoToPosition(() -> 0.0),
        // // shortClimb.shortGoToPosition(() -> ClimbConstants.l1ShortPosition))

        // // );
        // // }

        // // public static Command climbToL3(TallClimb tallClimb, ShortClimb
        // shortClimb) {
        // // // Tall arm goes to rung
        // // // Tall arm starts to pull down and Short arm starts to go up
        // // // By now after Short hooks on we are in L1
        // // // Now Tall arm goes up and Short arm goes down
        // // // After tall arm hooks on we are in L2
        // // // Now tall arm goes down and short arm goes up and hooks on without tall
        // arm
        // // // letting go
        // // // voilah we r in l3 now

        // // return new SequentialCommandGroup(
        // // new WaitCommand(0.3),
        // // new ParallelCommandGroup(
        // // tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
        // // shortClimb.shortGoToPosition(() -> 0.0)),
        // // new WaitCommand(0.3),
        // // new ParallelCommandGroup(
        // // tallClimb.tallGoToPosition(() -> 0.0),
        // // shortClimb.shortGoToPosition(() -> ClimbConstants.l1ShortPosition))

        // // );

        // // }
}