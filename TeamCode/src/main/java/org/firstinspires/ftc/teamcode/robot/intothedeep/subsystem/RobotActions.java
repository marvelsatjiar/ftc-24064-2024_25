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
    // 1. Add classes 2. delete unecessary delays 3. refactor delays
    public static class Transfer {
        public double
                SET_NEUTRAL_WRIST_WAIT = 0.25,
                SET_NEUTRAL_ARM_WAIT = 0.25,
                OUTTAKE_ROLLERS_WAIT = 0.25,
                SET_V4B_TRANSFER_WAIT = 0.25,
                CLAMP_CLAW_WAIT = 0.2,
                SET_COLLECTING_ARM_WAIT = 0.25;
    }

    public static class RetractionForTransfer {
        public double
                RETRACT_LIFT_WAIT = 1,
                SET_NEUTRAL_ARM_WAIT = 0.4,
                RETRACT_EXTENDO_WAIT = 0.7,
                SET_V4B_UP_WAIT = 0.2;
    }

    public static class ExtendIntake {
        public double EXTEND_EXTENDO_WAIT = 0.1;
    }

    public static class SetupBasket {
        public double EXTEND_LIFT_TO_SETUP_WAIT = 0.9;
    }

    public static class ScoreBasket {
        public double UNCLAMP_CLAW_TO_SCORE_WAIT = 0.2;
    }

    public static class SetupFrontWallPickup {
        public double
                RETRACT_TO_NEUTRAL_WAIT = 1,
                SET_V4B_TO_PICKUP_WAIT = 0.3,
                SET_WRIST_TO_PICKUP_WAIT = 0.1,
                SET_ARM_TO_PICKUP_WAIT = 0.1,
                SET_CLAW_TO_WALL_PICKUP_WAIT = 0.1;
    }

    public static class FrontWallSpecimenSetup {
        public double
                CLAMP_CLAW_TO_TAKE_SPECIMEN_WAIT = 0.2,
                SET_ARM_SPECIMEN_SETUP_WAIT = 0.1,
                SET_WRIST_SPECIMEN_SETUP_WAIT = 0.1,
                EXTEND_LIFT_FOR_SPECIMEN_SETUP_WAIT = 0.1;
    }

    public static class FrontWallSpecimenScore {
        public double
                SET_ARM_TO_SCORE_SPECIMEN_WAIT = 0.3;
    }

    public static class DropSamples {
        public double
                SET_WRIST_TO_DROP_SAMPLE_WAIT = 0.15,
                SET_ARM_TO_DROP_SAMPLE_WAIT = 0.15,
                UNCLAMP_CLAW_FOR_DROPOFF_WAIT = 0.3,
                RETRACT_TO_NEUTRAL_DELAY = 0.4;
    }

    public static class LevelTwoHang {
        public double
                EXTEND_LIFT_FOR_CLIMB_WAIT = 1,
                SET_ARM_NEUTRAL_WAIT = 1,
                RETRACT_LIFT_TO_CLIMB_WAIT = 3;
    }

    public static class OverhangSpecimen {
        public double SET_SPECIMEN_DELAY = 0.2;

    }

    public static Transfer TRANSFER = new Transfer();
    public static RetractionForTransfer RETRACTION_FOR_TRANSFER = new RetractionForTransfer();
    public static ExtendIntake EXTEND_INTAKE = new ExtendIntake();
    public static SetupBasket SETUP_BASKET = new SetupBasket();
    public static ScoreBasket SCORE_BASKET = new ScoreBasket();
    public static SetupFrontWallPickup SETUP_FRONT_WALL_PICKUP = new SetupFrontWallPickup();
    public static FrontWallSpecimenSetup FRONT_WALL_SPECIMEN_SETUP = new FrontWallSpecimenSetup();
    public static FrontWallSpecimenScore FRONT_WALL_SPECIMEN_SCORE = new FrontWallSpecimenScore();
    public static DropSamples DROP_SAMPLES = new DropSamples();
    public static LevelTwoHang LEVEL_TWO_HANG = new LevelTwoHang();
    public static OverhangSpecimen OVERHANG_SPECIMEN = new OverhangSpecimen();


    // DONE
    public static Action extendIntake(Extendo.Extension extension) {
        return new SequentialAction(
                setV4B(Intake.V4BAngle.UP, 0),
                setExtendo(extension, EXTEND_INTAKE.EXTEND_EXTENDO_WAIT),
                new InstantAction(() -> robot.currentState = Robot.State.EXTENDO_OUT)
        );
    }

    // DONE
    public static Action extendIntake(double angle) {
        return new SequentialAction(
                setV4B(Intake.V4BAngle.UP, 0),
                setExtendo(angle, EXTEND_INTAKE.EXTEND_EXTENDO_WAIT),
                new InstantAction(() -> robot.currentState = Robot.State.EXTENDO_OUT)
        );
    }



    // DONE
    public static Action retractForTransfer() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.TRANSFERRED,
                new SequentialAction(
                        new ParallelAction(
                                setRollers(0.7, 0),
                                new SequentialAction(
                                        setV4B(Intake.V4BAngle.UP, RETRACTION_FOR_TRANSFER.SET_V4B_UP_WAIT),
                                        setExtendo(Extendo.Extension.RETRACTED, RETRACTION_FOR_TRANSFER.RETRACT_EXTENDO_WAIT)
                                ),
                                new SequentialAction(
                                        setArm(Arm.ArmAngle.NEUTRAL, RETRACTION_FOR_TRANSFER.SET_NEUTRAL_ARM_WAIT),
                                        setLift(Lift.Ticks.RETRACTED, RETRACTION_FOR_TRANSFER.RETRACT_LIFT_WAIT)
                                ),
                                setClaw(Claw.ClawAngles.DEPOSIT, 0)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.TO_BE_TRANSFERRED)
                )
        );
    }

    // DONE
    public static Action retractTransferAndSetupBasket() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SCORE_BASKET,
                new SequentialAction(
                        transferToClaw(),
                        setupScoreBasket(true)
                )
        );
    }

    // DONE
    public static Action transferToClaw() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.TRANSFERRED,
                new SequentialAction(
                        retractForTransfer(),
                        setWrist(Arm.WristAngle.COLLECTING, 0),
                        setArm(Arm.ArmAngle.COLLECTING, TRANSFER.SET_COLLECTING_ARM_WAIT),
                        setClaw(Claw.ClawAngles.CLAMPED, TRANSFER.CLAMP_CLAW_WAIT),
                        new ParallelAction(
                                setV4B(Intake.V4BAngle.TRANSFER, TRANSFER.SET_V4B_TRANSFER_WAIT),
                                setRollers(-0.75, TRANSFER.OUTTAKE_ROLLERS_WAIT),
                                setWrist(Arm.WristAngle.TRANSFERRED, TRANSFER.SET_NEUTRAL_WRIST_WAIT),
                                setArm(Arm.ArmAngle.NEUTRAL, TRANSFER.SET_NEUTRAL_ARM_WAIT)
                        ),
                        new ParallelAction(
                                setRollers(0, 0),
                                setV4B(Intake.V4BAngle.UP, 0)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.TRANSFERRED)
                )
        );
    }

    // DONE
    public static Action setupScoreBasket(boolean isHighBasket) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SCORE_BASKET,
                new SequentialAction(
                        setLift(isHighBasket ? Lift.Ticks.HIGH_BASKET : Lift.Ticks.LOW_BASKET, SETUP_BASKET.EXTEND_LIFT_TO_SETUP_WAIT),
                        setArm(Arm.ArmAngle.BASKET, 0),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_BASKET)
                )
        );
    }

    // DONE
    public static Action scoreBasket() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SCORED_SAMPLE_HIGH_BASKET,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.DEPOSIT, SCORE_BASKET.UNCLAMP_CLAW_TO_SCORE_WAIT),
                        new InstantAction(() -> robot.currentState = Robot.State.SCORED_SAMPLE_HIGH_BASKET)
                )
        );
    }

    // DONE
    public static Action setupFrontWallPickup() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.FRONT_WALL_PICKUP,
                new SequentialAction(
                        retractToNeutral(SETUP_FRONT_WALL_PICKUP.RETRACT_TO_NEUTRAL_WAIT),
                        setExtendo(Extendo.Extension.RETRACTED, RETRACTION_FOR_TRANSFER.RETRACT_EXTENDO_WAIT),
                        new ParallelAction(
                                setWrist(Arm.WristAngle.FRONT_WALL_PICKUP, SETUP_FRONT_WALL_PICKUP.SET_WRIST_TO_PICKUP_WAIT),
                                setV4B(Intake.V4BAngle.FRONT_WALL_PICKUP, SETUP_FRONT_WALL_PICKUP.SET_V4B_TO_PICKUP_WAIT),
                                setWallPickupClaw(SETUP_FRONT_WALL_PICKUP.SET_CLAW_TO_WALL_PICKUP_WAIT)
                        ),
                        setArm(Arm.ArmAngle.FRONT_WALL_PICKUP, SETUP_FRONT_WALL_PICKUP.SET_ARM_TO_PICKUP_WAIT),
                        new InstantAction(() -> robot.currentState = Robot.State.FRONT_WALL_PICKUP)
                )
        );
    }


    // NON OVERHANG SETUP
    public static Action takeSpecimenFromFrontWallPickup() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_FRONT_SPECIMEN_FROM_WALL,
                new SequentialAction(
                        setupFrontWallPickup(),
                        setClaw(Claw.ClawAngles.CLAMPED, FRONT_WALL_SPECIMEN_SETUP.CLAMP_CLAW_TO_TAKE_SPECIMEN_WAIT),
                        new ParallelAction(
                                setArm(Arm.ArmAngle.FRONT_WALL_SPECIMEN_SETUP, FRONT_WALL_SPECIMEN_SETUP.SET_ARM_SPECIMEN_SETUP_WAIT),
                                setWrist(Arm.WristAngle.FRONT_WALL_SPECIMEN_SETUP, FRONT_WALL_SPECIMEN_SETUP.SET_WRIST_SPECIMEN_SETUP_WAIT),
                                setLift(Lift.Ticks.FRONT_WALL_SPECIMEN_SETUP, FRONT_WALL_SPECIMEN_SETUP.EXTEND_LIFT_FOR_SPECIMEN_SETUP_WAIT)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_FRONT_SPECIMEN_FROM_WALL)
                )
        );
    }

    // NON OVERHANG SCORE
    public static Action scoreSpecimenFromFrontWallPickup() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        new ParallelAction(
                                setLift(Lift.Ticks.FRONT_WALL_SPECIMEN_SCORE, 0),
                                setWrist(Arm.WristAngle.FRONT_WALL_SPECIMEN_SCORE, 0),
                                setArm(Arm.ArmAngle.FRONT_WALL_SPECIMEN_SCORE, FRONT_WALL_SPECIMEN_SCORE.SET_ARM_TO_SCORE_SPECIMEN_WAIT)
                        ),
                        setClaw(Claw.ClawAngles.DEPOSIT, 0),
                        retractToNeutral(0),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

    public static Action takeAndSetupOverhangSpecimen() {
            return new Actions.SingleCheckAction(
                    () -> robot.currentState != Robot.State.SETUP_CHAMBER_FROM_FRONT,
                    new SequentialAction(
                            setupFrontWallPickup(),
                            setClaw(Claw.ClawAngles.CLAMPED, FRONT_WALL_SPECIMEN_SETUP.CLAMP_CLAW_TO_TAKE_SPECIMEN_WAIT),
                            new ParallelAction(
                                    setLift(Lift.Ticks.OVERHANG_SPECIMEN_SETUP, 0),
                                    setWrist(Arm.WristAngle.OVERHANG_SPECIMEN_SETUP, OVERHANG_SPECIMEN.SET_SPECIMEN_DELAY),
                                    setArm(Arm.ArmAngle.OVERHANG_SPECIMEN_SETUP, 0)
                            ),
                            new InstantAction(() -> robot.currentState = Robot.State.SETUP_CHAMBER_FROM_FRONT)
                    )
            );
    }

    public static Action stableTakeAndSetupOverhangSpecimen(double sleepSecondsBeforeSetup) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_CHAMBER_FROM_FRONT,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.CLAMPED, FRONT_WALL_SPECIMEN_SETUP.CLAMP_CLAW_TO_TAKE_SPECIMEN_WAIT),
                        setArm(Arm.ArmAngle.BEFORE_OVERHANG_SPECIMEN, sleepSecondsBeforeSetup),
                        new InstantAction(() -> robot.currentState = Robot.State.FRONT_WALL_PICKUP),
                        takeAndSetupOverhangSpecimen()
                )
        );
    }

    public static Action stableTakeFromWallPickup(double sleepSecondsBeforeSetup) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_CHAMBER_FROM_FRONT,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.CLAMPED, FRONT_WALL_SPECIMEN_SETUP.CLAMP_CLAW_TO_TAKE_SPECIMEN_WAIT),
                        setArm(Arm.ArmAngle.FRONT_WALL_SPECIMEN_SETUP, sleepSecondsBeforeSetup),
                        new InstantAction(() -> robot.currentState = Robot.State.FRONT_WALL_PICKUP),
                        takeSpecimenFromFrontWallPickup()
                )
        );
    }

    // DONE - MAKE INTO DONE
    public static Action scoreOverhangSpecimen() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SCORE_OVERHANG_SPECIMEN,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.DEPOSIT, 0),
                        new InstantAction(() -> robot.currentState = Robot.State.SCORE_OVERHANG_SPECIMEN)
                )
        );
    }

    public static Action retractToNeutral(double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        new ParallelAction(
                                setClaw(Claw.ClawAngles.DEPOSIT, sleepSeconds),
                                setArm(Arm.ArmAngle.NEUTRAL, 0),
                                setWrist(Arm.WristAngle.COLLECTING, 0),
                                setLift(Lift.Ticks.RETRACTED, 0)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

//    public static Action alignRobotWithSensor(AutoAligner.TargetDistance targetDistance, GamepadKeys.Button button) {
//        return new SequentialAction(
//                new InstantAction(() -> robot.autoAligner.setTargetDistance(targetDistance)),
//                new Actions.RunnableAction(() -> MainTeleOp.gamepadEx1.isDown(button)),
//                new InstantAction(() -> robot.autoAligner.setTargetDistance(AutoAligner.TargetDistance.INACTIVE))
//        );
//    }

    // DONE
    public static Action setupLevelTwoHang() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_LEVEL_TWO_HANG,
                new SequentialAction(
                        new ParallelAction(
                                setLift(Lift.Ticks.LEVEL_TWO_CLIMB_SETUP, LEVEL_TWO_HANG.EXTEND_LIFT_FOR_CLIMB_WAIT),
                                setArm(Arm.ArmAngle.NEUTRAL, LEVEL_TWO_HANG.SET_ARM_NEUTRAL_WAIT)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_LEVEL_TWO_HANG)
                )
        );
    }

    // DONE
    public static Action climbLevelTwoHang() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.CLIMB_LEVEL_TWO_HANG,
                new SequentialAction(
                        setLift(Lift.Ticks.LEVEL_TWO_CLIMB, LEVEL_TWO_HANG.RETRACT_LIFT_TO_CLIMB_WAIT),
                        new InstantAction(() -> robot.currentState = Robot.State.CLIMB_LEVEL_TWO_HANG)
                )
        );
    }

    // DONE
    public static Action setupDropSample() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_DROP_SAMPLE,
                new SequentialAction(
                        new ParallelAction(
                                setWrist(Arm.WristAngle.BASKET, DROP_SAMPLES.SET_WRIST_TO_DROP_SAMPLE_WAIT),
                                setArm(Arm.ArmAngle.BASKET, DROP_SAMPLES.SET_ARM_TO_DROP_SAMPLE_WAIT)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_DROP_SAMPLE)
                )
        );
    }

    // DONE
    public static Action dropSample() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.DEPOSIT, DROP_SAMPLES.UNCLAMP_CLAW_FOR_DROPOFF_WAIT),
                        retractToNeutral(DROP_SAMPLES.RETRACT_TO_NEUTRAL_DELAY),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

    public static Action retractExtendo() {
        return new SequentialAction(
                setRollers(0.7 , 0),
                setV4B(Intake.V4BAngle.UP, RETRACTION_FOR_TRANSFER.SET_V4B_UP_WAIT),
                setExtendo(Extendo.Extension.RETRACTED, RETRACTION_FOR_TRANSFER.RETRACT_EXTENDO_WAIT),
                new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
        );
    }

/*

----------------------------------------------------------------------------------------------------

 */

    public static Action setV4B(Intake.V4BAngle angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.intake.getTargetV4BAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.intake.setTargetV4BAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setExtendo(Extendo.Extension extension, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.extendo.getTargetExtension() != extension,
                new ParallelAction(
                        new InstantAction(() -> robot.extendo.setTargetExtension(extension, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setExtendo(double angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.extendo.getTargetAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.extendo.setTargetAngle(angle, true)),
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

    public static Action setArm(Arm.ArmAngle angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getArmAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setArmAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setWrist(Arm.WristAngle angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getWristAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setWristAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setClaw(Claw.ClawAngles clawAngle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.claw.getClawAngle() != clawAngle,
                new ParallelAction(
                        new InstantAction(() -> robot.claw.setAngle(clawAngle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setSweeper(Sweeper.SweeperAngles sweeperAngle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.sweeper.getSweeperAngle() != sweeperAngle,
                new ParallelAction(
                        new InstantAction(() -> robot.sweeper.setAngle(sweeperAngle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setWallPickupClaw(double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.claw.getClawAngle() != Claw.ClawAngles.WALL_PICKUP,
                new ParallelAction(
                        new InstantAction(() -> robot.claw.setAngle(Claw.ClawAngles.WALL_PICKUP, true)),
                        new SleepAction(sleepSeconds)
                ));
    }

    public static Action setRollers(double power, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.intake.setRollerPower(power, true)),
                new SleepAction(sleepSeconds)
        );
    }
}


