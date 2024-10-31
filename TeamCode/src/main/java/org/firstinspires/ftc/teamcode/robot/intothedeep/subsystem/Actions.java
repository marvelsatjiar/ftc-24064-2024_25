package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

public class Actions {
    public SequentialAction retractForTransfer() {
        return new SequentialAction(
                lockSubsytems(),
                new ParallelAction(
                        new SequentialAction(
                                setV4B(Intake.V4BAngles.UP),
                                setExtendo(Extendo.LINKAGE_MIN_ANGLE)
                        ),
                        setLift(Lift.SlideTicks.RETRACTED),
                        setArm(Arm.Position.COLLECTING),
                        setClaw(false)
                ),
                unlockSubsytems()
        );
    }

    public SequentialAction transferToClaw() {
        return new SequentialAction(
                retractForTransfer(),
                lockSubsytems(),
                setClaw(true),
                new ParallelAction(
                        setRollers(-0.75),
                        setArm(Arm.Position.HIGH_BASKET)
                ),
                setRollers(0),
                unlockSubsytems()
        );
    }

    private Action lockSubsytems() {
        return new InstantAction(() -> {
            robot.intake.isLocked = true;
            robot.extendo.isLocked = true;
            robot.lift.isLocked = true;
            robot.arm.isLocked = true;
            robot.claw.isLocked = true;
            robot.intake.isRollerLocked = true;
        });
    }

    private Action unlockSubsytems() {
        return new InstantAction(() -> {
            robot.intake.isLocked = false;
            robot.extendo.isLocked = false;
            robot.lift.isLocked = false;
            robot.arm.isLocked = false;
            robot.claw.isLocked = false;
            robot.intake.isRollerLocked = false;
        });
    }

    private Action setV4B(Intake.V4BAngles angles) {
        if (robot.intake.getTargetAngle() == angles) return new NullAction();

        return new ParallelAction(
                new InstantAction(() -> robot.intake.setTarget(angles, true)),
                new SleepAction(0.5)
        );
    }

    private Action setExtendo(double angle) {
        if (robot.extendo.getLinkageTargetAngle() == angle) return new NullAction();

        return new ParallelAction(
                new InstantAction(() -> robot.extendo.setAngle(angle, true)),
                new SleepAction(0.5)
        );
    }

    private Action setLift(Lift.SlideTicks slideState) {
        if (robot.lift.getSetPoint() == slideState) return new NullAction();

        return new InstantAction(() -> robot.lift.setPosition(slideState, true));
    }

    private Action setArm(Arm.Position armPos) {
        if (robot.arm.getTargetPosition() == armPos) return new NullAction();

        return new InstantAction(() -> robot.arm.setTarget(armPos, true));
    }

    private Action setClaw(boolean isClosed) {
        return new InstantAction(() -> robot.claw.setClamped(isClosed, true));
    }

    private Action setRollers(double power) {
        return new InstantAction(() -> robot.intake.setServoPower(power, true));
    }
}
