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

    public static Action setupScoreHighBasket() {
        if (robot.currentState == Robot.State.SETUP_SCORE_HIGH_BASKET) return new NullAction();
        return new SequentialAction(
                setLift(Lift.Ticks.HIGH_BASKET),
                setArm(Arm.Position.HIGH_BASKET),
                new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_HIGH_BASKET)
        );
    }

    public static Action setupScoreHighChamberUpwards() {
        if (robot.currentState == Robot.State.SETUP_SCORE_HIGH_CHAMBER_UPWARDS) return new NullAction();
        return new SequentialAction(
                setLift(Lift.Ticks.HIGH_CHAMBER_UPWARDS),
                setArm(Arm.Position.HIGH_CHAMBER_UPWARDS),
                new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_HIGH_CHAMBER_UPWARDS)
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
