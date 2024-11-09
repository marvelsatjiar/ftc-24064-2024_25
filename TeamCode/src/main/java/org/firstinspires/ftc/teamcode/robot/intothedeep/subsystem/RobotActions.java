package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.auto.Actions;

public class RobotActions {
    public static Action extendIntake() {
        if (robot.currentState == Robot.State.SETUP_INTAKE) return new NullAction();
        return new SequentialAction(
                setV4B(Intake.V4BAngle.UP),
                setExtendo(Extendo.State.EXTENDED),
                new InstantAction(() -> robot.currentState = Robot.State.SETUP_INTAKE)
        );
    }


    public static Action retractForTransfer() {
        if (robot.currentState == Robot.State.TO_BE_TRANSFERRED) return new NullAction();
        return new SequentialAction(
                new ParallelAction(
                        new SequentialAction(
                                setV4B(Intake.V4BAngle.UP),
                                setExtendo(Extendo.State.RETRACTED)
                        ),
                        new SequentialAction(
                                setArm(Arm.ArmAngle.NEUTRAL),
                                setLift(Lift.Ticks.RETRACTED)
                        ),
                        setClaw(false)
                ),
                new InstantAction(() -> robot.currentState = Robot.State.TO_BE_TRANSFERRED)
        );
    }

    public static Action transferToClaw() {
        if (robot.currentState == Robot.State.TRANSFERRED) return new NullAction();
        return new SequentialAction(
                retractForTransfer(),
                setWrist(Arm.WristAngle.COLLECTING),
                setArm(Arm.ArmAngle.COLLECTING),
                setClaw(true),
                new ParallelAction(
                        setRollers(-0.75),
                        setArm(Arm.ArmAngle.NEUTRAL),
                        setWrist(Arm.WristAngle.NEUTRAL)
                ),
                setRollers(0),
                new InstantAction(() -> robot.currentState = Robot.State.TRANSFERRED)
        );
    }

    public static Action setupScoreBasket(boolean isHighBasket) {
        if (robot.currentState == Robot.State.SETUP_SCORE_BASKET) return new NullAction();
        return new SequentialAction(
                setLift(isHighBasket ? Lift.Ticks.HIGH_BASKET : Lift.Ticks.LOW_BASKET),
                setArm(Arm.ArmAngle.BASKET),
                new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_BASKET)
        );
    }

    public static Action setupScoreChamber(boolean isHighChamber) {
        if (robot.currentState == Robot.State.SETUP_SCORE_CHAMBER) return new NullAction();
        return new SequentialAction(
                setLift(isHighChamber? Lift.Ticks.HIGH_CHAMBER : Lift.Ticks.LOW_CHAMBER),
                setArm(Arm.ArmAngle.CHAMBER),
                new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_CHAMBER)
        );
    }

    public static Action scoreBasketAndRetract() {
        if (robot.currentState == Robot.State.SCORED) return new NullAction();
        return new SequentialAction(
                setClaw(false),
                setArm(Arm.ArmAngle.COLLECTING),
                setLift(Lift.Ticks.RETRACTED)
        );
    }

    public static Action scoreChamberandRetract() {
        if (robot.currentState == Robot.State.SCORED) return new NullAction();
        return new SequentialAction(
                new ParallelAction(
                        setClaw(false),
                        setLift(Lift.Ticks.EXTENDED)
                        ),
                setArm(Arm.ArmAngle.COLLECTING),
                setLift(Lift.Ticks.RETRACTED)
        );
    }

/*

----------------------------------------------------------------------------------------------------

 */

    private static Action setV4B(Intake.V4BAngle angle) {
        return new Actions.SingleCheckAction(
                () -> robot.intake.getTargetV4BAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.intake.setTargetV4BAngle(angle, true)),
                        new SleepAction(1)
                )
        );
    }

    private static Action setExtendo(Extendo.State state) {
        return new Actions.SingleCheckAction(
                () -> robot.extendo.getState() != state,
                new ParallelAction(
                        new InstantAction(() -> robot.extendo.setTargetState(state, true)),
                        new SleepAction(1)
                )
        );
    }

    private static Action setLift(Lift.Ticks ticks) {
        return new Actions.SingleCheckAction(
                () -> robot.lift.getTargetTicks() != ticks,
                new ParallelAction(
                        new InstantAction(() -> robot.lift.setTargetTicks(ticks, true)),
                        new SleepAction(1)
                )
        );
    }

    private static Action setArm(Arm.ArmAngle angle) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getArmAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setArmAngle(angle, true)),
                        new SleepAction(1)
                )
        );
    }

    private static Action setWrist(Arm.WristAngle angle) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getWristAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setWristAngle(angle, true)),
                        new SleepAction(1)
                )
        );
    }

    private static Action setClaw(boolean isClamped) {
        return new Actions.SingleCheckAction(
                () -> robot.claw.getClamped() != isClamped,
                new ParallelAction(
                        new InstantAction(() -> robot.claw.setClamped(isClamped, true)),
                        new SleepAction(1)
                )
        );
    }

    private static Action setRollers(double power) {
        return new ParallelAction(
                new InstantAction(() -> robot.intake.setRollerPower(power, true)),
                new SleepAction(1)
        );
    }
}
