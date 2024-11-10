package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.auto.Actions;

@Config
public class RobotActions {
    private static final double LIFT_EXTEND_SETUP_SCORE_BASKET = 2;
    private static final double V4B_UP_EXTEND_INTAKE = 0.25;
    private static final double LIFT_RETRACTED_SCORE_BASKET_AND_RETRACT = 2;
    private static final double WRIST_NEUTRAL_SCORE_BASKET_AND_RETRACT = 2;
    private static final double ARM_NEUTRAL_SCORE_BASKET_AND_RETRACT = 2;
    private static final double CLAW_UNCLAMPED_SCORE_BASKET_AND_RETRACT = 0.5;
    private static final double ARM_CHAMBER_SETUP_SCORE_CHAMBER = 1;
    private static final double LIFT_EXTEND_SETUP_SCORE_CHAMBER = 1;
    private static final double ARM_BASKET_SETUP_SCORE_BASKET = 0.7;
    private static final double ROLLERS_STOP_TRANSFER_TO_CLAW = 0;
    private static final double WRIST_NEUTRAL_TRANSFER_TO_CLAW = 0.75;
    private static final double ARM_NEUTRAL_TRANSFER_TO_CLAW = 0.75;
    private static final double ROLLERS_OUTTAKE_TRANSFER_TO_CLAW = 0.75;
    private static final double CLAW_CLAMPED_TRANSFER_TO_CLAW = 0.2;
    private static final double ARM_COLLECTING_TRANSFER_TO_CLAW = 0.7;
    private static final double WRIST_COLLECTING_TRANSFER_TO_CLAW = 0.5;
    private static final double CLAW_UNCLAMPED_RETRACT_FOR_TRANSFER = 0;
    private static final double LIFT_RETRACTED_RETRACT_FOR_TRANSFER = 1;
    private static final double ARM_NEUTRAL_RETRACT_FOR_TRANSFER = 0.4;
    private static final double EXTENDO_RETRACTED_RETRACT_FOR_TRANSFER = 0.7;
    private static final double V4B_UP_RETRACT_FOR_TRANSFER = 0.33;
    private static final double EXTENDO_EXTENDED_EXTEND_INTAKE = 0;

    public static Action extendIntake() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_INTAKE,
                new SequentialAction(
                        setV4B(Intake.V4BAngle.UP, V4B_UP_EXTEND_INTAKE),
                        setExtendo(Extendo.State.EXTENDED, EXTENDO_EXTENDED_EXTEND_INTAKE),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_INTAKE)
                )
        );
    }

    public static Action retractForTransfer() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.TO_BE_TRANSFERRED,
                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        setV4B(Intake.V4BAngle.UP, V4B_UP_RETRACT_FOR_TRANSFER),
                                        setExtendo(Extendo.State.RETRACTED, EXTENDO_RETRACTED_RETRACT_FOR_TRANSFER)
                                ),
                                new SequentialAction(
                                        setArm(Arm.ArmAngle.NEUTRAL, ARM_NEUTRAL_RETRACT_FOR_TRANSFER),
                                        setLift(Lift.Ticks.RETRACTED, LIFT_RETRACTED_RETRACT_FOR_TRANSFER)
                                ),
                                setClaw(false, CLAW_UNCLAMPED_RETRACT_FOR_TRANSFER)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.TO_BE_TRANSFERRED)
                )
        );
    }

    public static Action transferToClaw() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.TRANSFERRED,
                new SequentialAction(
                        retractForTransfer(),
                        setWrist(Arm.WristAngle.COLLECTING, WRIST_COLLECTING_TRANSFER_TO_CLAW),
                        setArm(Arm.ArmAngle.COLLECTING, ARM_COLLECTING_TRANSFER_TO_CLAW),
                        setClaw(true, CLAW_CLAMPED_TRANSFER_TO_CLAW),
                        new ParallelAction(
                                setRollers(-0.75, ROLLERS_OUTTAKE_TRANSFER_TO_CLAW),
                                setArm(Arm.ArmAngle.NEUTRAL, ARM_NEUTRAL_TRANSFER_TO_CLAW),
                                setWrist(Arm.WristAngle.NEUTRAL, WRIST_NEUTRAL_TRANSFER_TO_CLAW)
                        ),
                        setRollers(0, ROLLERS_STOP_TRANSFER_TO_CLAW),

                        new InstantAction(() -> robot.currentState = Robot.State.TRANSFERRED)
                )
        );
    }

    public static Action setupScoreBasket(boolean isHighBasket) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SCORE_BASKET,
                new SequentialAction(
                        setLift(isHighBasket ? Lift.Ticks.HIGH_BASKET : Lift.Ticks.LOW_BASKET, LIFT_EXTEND_SETUP_SCORE_BASKET),
                        setArm(Arm.ArmAngle.BASKET, ARM_BASKET_SETUP_SCORE_BASKET),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_BASKET)
                )
        );
    }

    // TODO
    public static Action setupScoreChamber(boolean isHighChamber) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SCORE_CHAMBER,
                new SequentialAction(
                        setLift(isHighChamber ? Lift.Ticks.HIGH_CHAMBER_SETUP : Lift.Ticks.HIGH_CHAMBER_SCORE, LIFT_EXTEND_SETUP_SCORE_CHAMBER),
                        setArm(Arm.ArmAngle.CHAMBER, ARM_CHAMBER_SETUP_SCORE_CHAMBER),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_CHAMBER)
                )
        );
    }

    public static Action scoreBasketAndRetract(boolean isHighBasket) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        setupScoreBasket(isHighBasket),
                        setClaw(false, CLAW_UNCLAMPED_SCORE_BASKET_AND_RETRACT),
                        new ParallelAction(
                                setArm(Arm.ArmAngle.NEUTRAL, ARM_NEUTRAL_SCORE_BASKET_AND_RETRACT),
                                setWrist(Arm.WristAngle.NEUTRAL, WRIST_NEUTRAL_SCORE_BASKET_AND_RETRACT),
                                setLift(Lift.Ticks.RETRACTED, LIFT_RETRACTED_SCORE_BASKET_AND_RETRACT)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

    // TODO
    public static Action scoreChamberAndRetract() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SCORED,
                new SequentialAction(
//                        new ParallelAction(
//                                setClaw(false),
//                                setLift(Lift.Ticks.EXTENDED)
//                        ),
//                        setArm(Arm.ArmAngle.COLLECTING),
//                        setLift(Lift.Ticks.RETRACTED),
//                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

/*

----------------------------------------------------------------------------------------------------

 */

    private static Action setV4B(Intake.V4BAngle angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.intake.getTargetV4BAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.intake.setTargetV4BAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    private static Action setExtendo(Extendo.State state, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.extendo.getState() != state,
                new ParallelAction(
                        new InstantAction(() -> robot.extendo.setTargetState(state, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    private static Action setLift(Lift.Ticks ticks, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.lift.getTargetTicks() != ticks,
                new ParallelAction(
                        new InstantAction(() -> robot.lift.setTargetTicks(ticks, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    private static Action setArm(Arm.ArmAngle angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getArmAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setArmAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    private static Action setWrist(Arm.WristAngle angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getWristAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setWristAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    private static Action setClaw(boolean isClamped, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.claw.getClamped() != isClamped,
                new ParallelAction(
                        new InstantAction(() -> robot.claw.setClamped(isClamped, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    private static Action setRollers(double power, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.intake.setRollerPower(power, true)),
                new SleepAction(sleepSeconds)
        );
    }
}
