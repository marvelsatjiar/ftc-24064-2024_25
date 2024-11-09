package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

public class RobotActions {
    public static Action extendIntake() {
        if (robot.currentState == Robot.State.SETUP_INTAKE) return new NullAction();
        return new SequentialAction(
                setV4B(Intake.V4BAngle.UP),
                setExtendo(Extendo.State.EXTENDED),
                setV4B(Intake.V4BAngle.DOWN),
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
                                setArm(Arm.Position.NEUTRAL),
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
                setArm(Arm.Position.COLLECTING),
                setClaw(true),
                new ParallelAction(
                        setRollers(-0.75),
                        setArm(Arm.Position.NEUTRAL)
                ),
                setRollers(0),
                new InstantAction(() -> robot.currentState = Robot.State.TRANSFERRED)
        );
    }

    public static Action setupScoreBasket(boolean isHighBasket) {
        if (robot.currentState == Robot.State.SETUP_SCORE_BASKET) return new NullAction();
        return new SequentialAction(
                setLift(isHighBasket ? Lift.Ticks.HIGH_BASKET : Lift.Ticks.LOW_BASKET),
                setArm(Arm.Position.BASKET),
                new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_BASKET)
        );
    }

    public static Action setupScoreChamber(boolean isHighChamber) {
        if (robot.currentState == Robot.State.SETUP_SCORE_CHAMBER) return new NullAction();
        return new SequentialAction(
                setLift(isHighChamber? Lift.Ticks.HIGH_CHAMBER : Lift.Ticks.LOW_CHAMBER),
                setArm(Arm.Position.CHAMBER),
                new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_CHAMBER)
        );
    }

    public static Action scoreBasketandRetract() {
        if (robot.currentState == Robot.State.SCORED) return new NullAction();
        return new SequentialAction(
                setClaw(false),
                setArm(Arm.Position.COLLECTING),
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
                setArm(Arm.Position.COLLECTING),
                setLift(Lift.Ticks.RETRACTED)
        );
    }

/*

----------------------------------------------------------------------------------------------------

 */

    private static Action setV4B(Intake.V4BAngle angle) {
        if (robot.intake.getTargetV4BAngle() == angle) return new NullAction();

        return new ParallelAction(
                new InstantAction(() -> robot.intake.setTargetV4BAngle(angle, true)),
                new SleepAction(3)
        );
    }

    private static Action setExtendo(Extendo.State state) {
        if (robot.extendo.getState() == state) return new NullAction();

        return new ParallelAction(
                new InstantAction(() -> robot.extendo.setTargetAngle(state, true)),
                new SleepAction(4)
        );
    }

    private static Action setLift(Lift.Ticks ticks) {
        if (robot.lift.getTargetTicks() == ticks) return new NullAction();

        return new ParallelAction(
                new InstantAction(() -> robot.lift.setTargetTicks(ticks, true)),
                new SleepAction(3)
        );
    }

    private static Action setArm(Arm.Position position) {
        if (robot.arm.getTargetPosition() == position) return new NullAction();

        return new ParallelAction(
                new InstantAction(() -> robot.arm.setTargetPosition(position, true)),
                new SleepAction(3)
        );
    }

    private static Action setClaw(boolean isClamped) {
        if (robot.claw.getClamped() == isClamped) return new NullAction();

        return new ParallelAction(
                new InstantAction(() -> robot.claw.setClamped(isClamped, true)),
                new SleepAction(2)
        );
    }

    private static Action setRollers(double power) {
        return new ParallelAction(
                new InstantAction(() -> robot.intake.setRollerPower(power, true)),
                new SleepAction(1)
        );
    }
}
