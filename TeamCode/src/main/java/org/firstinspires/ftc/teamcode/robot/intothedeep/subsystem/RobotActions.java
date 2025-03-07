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
                setArmstendoTransferWait = 0.3,
                setV4BTransferWait = 0.25,
                retractArmstendoWait = 0.2,
                clampClawWait = 0.2,
                setCollectingArmWait = 0.25;
    }

    public static class RetractionForTransfer {
        public double
                retractLiftWait = 1,
                setNeutralArmWait = 0.4,
                retractExtendoWait = 0.7,
                setV4BUpWait = 0.2;
    }

    public static class ExtendIntake {
        public double extendExtendoWait = 0.1;
    }

    public static class SetupBasket {
        public double extendLiftToSetupWait = 0.9;
    }

    public static class ScoreBasket {
        public double unclampClawToScoreWait = 0;
    }

    public static class DropSamples {
        public double
                setWristToDropSampleWait = 0.15,
                setArmToDropSampleWait = 0.15,
                unclampClawForDropoffWait = 0.3,
                retractToNeutralDelay = 0.4;
    }

    public static class LevelTwoHang {
        public double
                extendLiftForClimbWait = 1,
                setArmNeutralWait = 1,
                retractLiftToClimbWait = 3;
    }

    public static class SetupWallPickup {
        public double
                setLiftWait = 0.4,
                setWristWait = 0.15,
                retractArmstendoWait = 0.3,
                setArmstendoWait = 0.3,
                setArmWait = 0.2;
    }

    public static class SetupSpecimen {
        public double
                clampClawWait = 0.2,
                setWristWait = 0.1,
                setArmWait = 0.25,
                setArmstendoWait = 0.3,
                setLiftWait = 0,
                extendArmstendoWait = 0.3;
    }

    public static class ScoreSpecimen {
        public double
                extendArmstendoWait = 0.3,
                unclampClawWait = 0.5;
    }

    public static class SetupSpecimenStable {
        public double clampClawWait = 0.2;
    }

    public static Transfer TRANSFER = new Transfer();
    public static RetractionForTransfer RETRACTION_FOR_TRANSFER = new RetractionForTransfer();
    public static ExtendIntake EXTEND_INTAKE = new ExtendIntake();
    public static SetupBasket SETUP_BASKET = new SetupBasket();
    public static ScoreBasket SCORE_BASKET = new ScoreBasket();
    public static DropSamples DROP_SAMPLES = new DropSamples();
    public static LevelTwoHang LEVEL_TWO_HANG = new LevelTwoHang();
    public static SetupWallPickup SETUP_WALL_PICKUP = new SetupWallPickup();
    public static SetupSpecimen SETUP_SPECIMEN = new SetupSpecimen();
    public static ScoreSpecimen SCORE_SPECIMEN = new ScoreSpecimen();
    public static SetupSpecimenStable SETUP_SPECIMEN_STABLE = new SetupSpecimenStable();


    // DONE
    public static Action extendIntake(Extendo.Extension extension) {
        return new SequentialAction(
                setV4B(Intake.V4BAngle.UP, 0),
                setExtendo(extension, EXTEND_INTAKE.extendExtendoWait),
                new InstantAction(() -> robot.currentState = Robot.State.EXTENDO_OUT)
        );
    }

    // DONE
    public static Action extendIntake(double angle) {
        return new SequentialAction(
                setV4B(Intake.V4BAngle.UP, 0),
                setExtendo(angle, EXTEND_INTAKE.extendExtendoWait),
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
                                        setV4B(Intake.V4BAngle.UP, RETRACTION_FOR_TRANSFER.setV4BUpWait),
                                        setExtendo(Extendo.Extension.RETRACTED, RETRACTION_FOR_TRANSFER.retractExtendoWait)
                                ),
                                new SequentialAction(
                                        setArm(Arm.ArmAngle.NEUTRAL, RETRACTION_FOR_TRANSFER.setNeutralArmWait),
                                        setLift(Lift.Ticks.RETRACTED, RETRACTION_FOR_TRANSFER.retractLiftWait)
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
                        transfer(),
                        setupBasket(true)
                )
        );
    }

    // TODO
    public static Action transfer() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.TRANSFERRED,
                new SequentialAction(
                        new ParallelAction(
                                retractForTransfer(),
                                setWrist(Arm.WristAngle.COLLECTING, 0)
                        ),
                        setArmstendo(Arm.Extension.TRANSFER, TRANSFER.setArmstendoTransferWait),
                        setArm(Arm.ArmAngle.COLLECTING, TRANSFER.setCollectingArmWait),
                        setClaw(Claw.ClawAngles.CLAMPED, TRANSFER.clampClawWait),
                        new ParallelAction(
                                setV4B(Intake.V4BAngle.TRANSFER, TRANSFER.setV4BTransferWait),
                                setWrist(Arm.WristAngle.TRANSFERRED, 0),
                                setArm(Arm.ArmAngle.NEUTRAL, 0)
                        ),
                        setArmstendo(Arm.Extension.RETRACTED, TRANSFER.retractArmstendoWait),
                        setV4B(Intake.V4BAngle.UP, 0),
                        new InstantAction(() -> robot.currentState = Robot.State.TRANSFERRED)
                )
        );
    }

    // DONE
    public static Action setupBasket(boolean isHighBasket) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SCORE_BASKET,
                new SequentialAction(
                        new ParallelAction(
                                setLift(isHighBasket ? Lift.Ticks.HIGH_BASKET : Lift.Ticks.LOW_BASKET, SETUP_BASKET.extendLiftToSetupWait),
                                setArmstendo(Arm.Extension.EXTENDED, 0)
                        ),
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
                        setClaw(Claw.ClawAngles.DEPOSIT, SCORE_BASKET.unclampClawToScoreWait),
                        new InstantAction(() -> robot.currentState = Robot.State.SCORED_SAMPLE_HIGH_BASKET)
                )
        );
    }

    public static Action retractAfterScoreBasket() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, 0.5),
                        RobotActions.retractToNeutral(0)
                )
        );
    }

    // TODO
    public static Action setupWallPickup() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.WALL_PICKUP,
                new SequentialAction(
                        setArmstendo(Arm.Extension.RETRACTED, SETUP_WALL_PICKUP.retractArmstendoWait),
                        setWrist(Arm.WristAngle.WALL_PICKUP, SETUP_WALL_PICKUP.setWristWait),
                        new ParallelAction(
                                setArm(Arm.ArmAngle.WALL_PICKUP, SETUP_WALL_PICKUP.setArmWait),
                                setClaw(Claw.ClawAngles.WALL_PICKUP, 0)
                        ),
                        setArmstendo(Arm.Extension.WALL_PICKUP, SETUP_WALL_PICKUP.setArmstendoWait),
                        setLift(Lift.Ticks.WALL_PICKUP, SETUP_WALL_PICKUP.setLiftWait),
                        new InstantAction(() -> robot.currentState = Robot.State.WALL_PICKUP)
                )
        );
    }

    // TODO
    public static Action setupSpecimen() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SPECIMEN,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.CLAMPED, SETUP_SPECIMEN.clampClawWait),
                        setWrist(Arm.WristAngle.GRAB_OFF_WALL, SETUP_SPECIMEN.setWristWait),
                        setArmstendo(Arm.Extension.RETRACTED, SETUP_SPECIMEN.setArmstendoWait),
                        setLift(Lift.Ticks.SETUP_SPECIMEN, SETUP_SPECIMEN.setLiftWait),
                        setArm(Arm.ArmAngle.SCORE_SPECIMEN, SETUP_SPECIMEN.setArmWait),
                        setWrist(Arm.WristAngle.SCORE_SPECIMEN, 0),
                        setArmstendo(Arm.Extension.SETUP_SPECIMEN, SETUP_SPECIMEN.extendArmstendoWait),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_SPECIMEN)
                )
        );
    }

    // TODO
    public static Action scoreSpecimen() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SCORE_SPECIMEN,
                new SequentialAction(
                        setArmstendo(Arm.Extension.EXTENDED, SCORE_SPECIMEN.extendArmstendoWait),
                        setClaw(Claw.ClawAngles.DEPOSIT, SCORE_SPECIMEN.unclampClawWait),
                        new InstantAction(() -> robot.currentState = Robot.State.SCORE_SPECIMEN)
                )
        );
    }

    // TODO
    public static Action setupSpecimenStable(double sleepSecondsBeforeSetup) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SPECIMEN,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.CLAMPED, SETUP_SPECIMEN_STABLE.clampClawWait),
                        setArmstendo(Arm.Extension.RETRACTED, sleepSecondsBeforeSetup),
                        setupSpecimen()
                )
        );
    }

    // DONE
    public static Action retractToNeutral(double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new ParallelAction(
                        setArmstendo(Arm.Extension.RETRACTED, sleepSeconds),
                        setClaw(Claw.ClawAngles.DEPOSIT, sleepSeconds),
                        setArm(Arm.ArmAngle.NEUTRAL, 0),
                        setWrist(Arm.WristAngle.COLLECTING, 0),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL),
                        setLift(Lift.Ticks.RETRACTED, 0)
                )
        );
    }

    // DONE
    public static Action setupLevelTwoHang() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_LEVEL_TWO_HANG,
                new SequentialAction(
                        new ParallelAction(
                                setLift(Lift.Ticks.LEVEL_TWO_CLIMB_SETUP, LEVEL_TWO_HANG.extendLiftForClimbWait),
                                setArm(Arm.ArmAngle.NEUTRAL, LEVEL_TWO_HANG.setArmNeutralWait)
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
                        setLift(Lift.Ticks.LEVEL_TWO_CLIMB, LEVEL_TWO_HANG.retractLiftToClimbWait),
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
                                setWrist(Arm.WristAngle.BASKET, DROP_SAMPLES.setWristToDropSampleWait),
                                setArm(Arm.ArmAngle.BASKET, DROP_SAMPLES.setArmToDropSampleWait)
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
                        setClaw(Claw.ClawAngles.DEPOSIT, DROP_SAMPLES.unclampClawForDropoffWait),
                        retractToNeutral(DROP_SAMPLES.retractToNeutralDelay),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

    // DONE
    public static Action retractExtendo() {
        return new SequentialAction(
                setRollers(0.7 , 0),
                setV4B(Intake.V4BAngle.UP, RETRACTION_FOR_TRANSFER.setV4BUpWait),
                setExtendo(Extendo.Extension.RETRACTED, RETRACTION_FOR_TRANSFER.retractExtendoWait),
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

    public static Action setArmstendo(Arm.Extension extension, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getArmstendoAngle() != extension,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setArmstendoAngle(extension, true)),
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


