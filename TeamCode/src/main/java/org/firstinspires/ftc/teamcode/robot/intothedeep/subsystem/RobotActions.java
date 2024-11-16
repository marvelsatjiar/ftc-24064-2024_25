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
    public static final double
            LIFT_EXTEND_SETUP_SCORE_BASKET = 2,
            V4B_UP_EXTEND_INTAKE = 0.25,
            CLAW_UNCLAMPED_SCORE_BASKET_AND_RETRACT = 0.5,
            ARM_CHAMBER_SETUP_CHAMBER_FROM_BACK = 1,
            LIFT_HIGH_CHAMBER_BACK_SETUP_CHAMBER_FROM_BACK = 1,
            ARM_BASKET_SETUP_SCORE_BASKET = 0.7,
            ROLLERS_STOP_TRANSFER_TO_CLAW = 0,
            WRIST_NEUTRAL_TRANSFER_TO_CLAW = 0.75,
            ARM_NEUTRAL_TRANSFER_TO_CLAW = 0.75,
            ROLLERS_OUTTAKE_TRANSFER_TO_CLAW = 0.75,
            CLAW_CLAMPED_TRANSFER_TO_CLAW = 0.2,
            ARM_COLLECTING_TRANSFER_TO_CLAW = 0.7,
            WRIST_COLLECTING_TRANSFER_TO_CLAW = 0.5,
            CLAW_UNCLAMPED_RETRACT_FOR_TRANSFER = 0,
            LIFT_RETRACTED_RETRACT_FOR_TRANSFER = 1,
            ARM_NEUTRAL_RETRACT_FOR_TRANSFER = 0.4,
            EXTENDO_RETRACTED_RETRACT_FOR_TRANSFER = 0.7,
            V4B_UP_RETRACT_FOR_TRANSFER = 0.33,
            EXTENDO_EXTENDED_EXTEND_INTAKE = 0,
            WRIST_CHAMBER_SETUP_CHAMBER_FROM_BACK = 1,
            LIFT_HIGH_CHAMBER_BACK_SCORE_CHAMBER_FROM_BACK_AND_RETRACT = 1.25,
            CLAW_UNCLAMPED_SCORE_CHAMBER_FROM_BACK_AND_RETRACT = 1.25,
            LIFT_HIGH_CHAMBER_FRONT_SETUP_CHAMBER_FROM_FRONT = 2,
            ARM_CHAMBER_SETUP_FRONT_CHAMBER_FROM_FRONT = 1,
            WRIST_CHAMBER_SETUP_FRONT_CHAMBER_FROM_FRONT = 1, 
            LIFT_HIGH_CHAMBER_FRONT_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT = 0.85,
            ARM_CHAMBER_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT = 1.15,
            WRIST_CHAMBER_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT = 1.15,
            CLAW_UNCLAMPED_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT = 1.15,
            RETRACT_TO_NEUTRAL_SCORE_BASKET_AND_RETRACT = 1,
            RETRACT_TO_NEUTRAL_SCORE_CHAMBER_FROM_BACK_AND_RETRACT = 1,
            RETRACT_TO_NEUTRAL_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT = 1,
            LIFT_SETUP_WALL_PICKUP = 1,
            ARM_SETUP_WALL_PICKUP = 0.65,
            WRIST_SETUP_WALL_PICKUP = 0.65,
            CLAW_CLAMPED_WALL_PICKUP = 1,
            ARM_NEUTRAL_WALL_PICKUP = 1,
            WRIST_COLLECTING_WALL_PICKUP = 1;

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
                                setWrist(Arm.WristAngle.COLLECTING, WRIST_NEUTRAL_TRANSFER_TO_CLAW)
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

    public static Action scoreBasketAndRetract(boolean isHighBasket) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        setupScoreBasket(isHighBasket),
                        setClaw(false, CLAW_UNCLAMPED_SCORE_BASKET_AND_RETRACT),
                        retractToNeutral(RETRACT_TO_NEUTRAL_SCORE_BASKET_AND_RETRACT),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

    public static Action setupChamberFromBack() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_CHAMBER_FROM_BACK,
                new SequentialAction(
                        new ParallelAction(
                                setLift(Lift.Ticks.HIGH_CHAMBER_SETUP_BACK, LIFT_HIGH_CHAMBER_BACK_SETUP_CHAMBER_FROM_BACK),
                                setWrist(Arm.WristAngle.CHAMBER_BACK, WRIST_CHAMBER_SETUP_CHAMBER_FROM_BACK),
                                setArm(Arm.ArmAngle.CHAMBER_FRONT, ARM_CHAMBER_SETUP_CHAMBER_FROM_BACK)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_CHAMBER_FROM_BACK)
                )
        );
    }

    public static Action scoreChamberFromBackAndRetract() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        setupChamberFromBack(),
                        new ParallelAction(
                                setLift(Lift.Ticks.HIGH_CHAMBER_SCORE_BACK, LIFT_HIGH_CHAMBER_BACK_SCORE_CHAMBER_FROM_BACK_AND_RETRACT),
                                setClaw(false, CLAW_UNCLAMPED_SCORE_CHAMBER_FROM_BACK_AND_RETRACT)
                        ),
                        retractToNeutral(RETRACT_TO_NEUTRAL_SCORE_CHAMBER_FROM_BACK_AND_RETRACT),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

    public static Action setupChamberFromFront() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_CHAMBER_FROM_FRONT,
                new SequentialAction(
                        setLift(Lift.Ticks.HIGH_CHAMBER_SETUP_FRONT, LIFT_HIGH_CHAMBER_FRONT_SETUP_CHAMBER_FROM_FRONT),
                        new ParallelAction(
                                setWrist(Arm.WristAngle.CHAMBER_FRONT, WRIST_CHAMBER_SETUP_FRONT_CHAMBER_FROM_FRONT),
                                setArm(Arm.ArmAngle.CHAMBER_FRONT, ARM_CHAMBER_SETUP_FRONT_CHAMBER_FROM_FRONT)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_CHAMBER_FROM_FRONT)
                )
        );
    }

    public static Action scoreChamberFromFrontAndRetract() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        setupChamberFromFront(),
                        setLift(Lift.Ticks.HIGH_CHAMBER_SCORE_FRONT, LIFT_HIGH_CHAMBER_FRONT_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT),
                        new ParallelAction(
                                setArm(Arm.ArmAngle.CHAMBER_FRONT, ARM_CHAMBER_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT),
                                setWrist(Arm.WristAngle.CHAMBER_FRONT, WRIST_CHAMBER_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT),
                                setClaw(false, CLAW_UNCLAMPED_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT)
                        ),
                        retractToNeutral(RETRACT_TO_NEUTRAL_SCORE_CHAMBER_FROM_FRONT_AND_RETRACT),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

    public static Action setupWallPickup() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_WALL_PICKUP,
                new SequentialAction(
                        setLift(Lift.Ticks.WALL_PICKUP, LIFT_SETUP_WALL_PICKUP),
                        new ParallelAction(
                                setArm(Arm.ArmAngle.WALL_PICKUP, ARM_SETUP_WALL_PICKUP),
                                setWrist(Arm.WristAngle.WALL_PICKUP, WRIST_SETUP_WALL_PICKUP)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_WALL_PICKUP)
                )
        );
    }

    public static Action wallPickup() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.WALL_PICKUP,
                new SequentialAction(
                        setupWallPickup(),
                        new ParallelAction(
                                setClaw(true, CLAW_CLAMPED_WALL_PICKUP),
                                setArm(Arm.ArmAngle.NEUTRAL, ARM_NEUTRAL_WALL_PICKUP),
                                setWrist(Arm.WristAngle.COLLECTING, WRIST_COLLECTING_WALL_PICKUP)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.WALL_PICKUP)
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

    private static Action retractToNeutral(double sleepSeconds) {
        return new ParallelAction(
                setArm(Arm.ArmAngle.NEUTRAL, sleepSeconds),
                setWrist(Arm.WristAngle.COLLECTING, 0),
                setLift(Lift.Ticks.RETRACTED, 0)
        );
    }
}
