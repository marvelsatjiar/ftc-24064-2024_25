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
                setNeutralWristWait = 0.25,
                setNeutralArmWait = 0.25,
                outtakeRollersWait = 0.25,
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
        public double unclampClawToScoreWait = 0.2;
    }

    public static class SetupFrontWallPickup {
        public double
                retractToNeutralDelay = 1,
                setV4BToPickupWait = 0.3,
                setWristToPickupWait = 0.1,
                setArmToPickupWait = 0.1,
                setClawToWallPickupWait = 0.1;
    }

    public static class FrontWallSpecimenSetup {
        public double
                clampClawToTakeSpecimenWait = 0.2,
                setArmSpecimenSetupWait = 0.1,
                setWristSpecimenSetupWait = 0.1,
                extendLiftForSpecimenSetupWait = 0.1;
    }

    public static class FrontWallSpecimenScore {
        public double
                setArmToScoreSpecimenWait = 0.3;
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

    public static class OverhangSpecimen {
        public double setSpecimenDelay = 0.2;
    }

    public static class BackWallPickup {
        public double
                setLiftToPickupFromBackWait = 0.4,
                setArmToPickupFromBackWait = 0.2;
    }

    public static class setupSpecimenWithArmstendo {
        public double
                clampClawToPickupFromBackWait = 0.2,
                extendLiftBeforeBackWallSpecimenWait = 0.4,
                extendArmstendoBeforeBackWallSpecimenWait = 0.3;
    }

    public static class scoreSpecimenWithArmstendo {
        public double
                extendArmstendoToScoreSpecimenWait = 0.3,
                unclampClawToScoreSpecimenWait = 0.5;
    }

    public static class StableTakeFromBackWallSpecimen {
        public double clampClawToTakeSpecimenWait = 0.2;
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
    public static BackWallPickup BACK_WALL_PICKUP = new BackWallPickup();
    public static setupSpecimenWithArmstendo SETUP_SPECIMEN_WITH_ARMSTENDO = new setupSpecimenWithArmstendo();
    public static scoreSpecimenWithArmstendo SCORE_SPECIMEN_WITH_ARMSTENDO = new scoreSpecimenWithArmstendo();
    public static StableTakeFromBackWallSpecimen STABLE_TAKE_FROM_BACK_WALL_PICKUP = new StableTakeFromBackWallSpecimen();


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
                        new ParallelAction(
                                setArm(Arm.ArmAngle.COLLECTING, TRANSFER.setCollectingArmWait),
                                setArmstendo(Arm.Extension.TRANSFER, 0)
                        ),
                        setClaw(Claw.ClawAngles.CLAMPED, TRANSFER.clampClawWait),
                        new ParallelAction(
                                setV4B(Intake.V4BAngle.TRANSFER, TRANSFER.setV4BTransferWait),
                                setRollers(-0.75, TRANSFER.outtakeRollersWait),
                                setWrist(Arm.WristAngle.TRANSFERRED, TRANSFER.setNeutralWristWait),
                                setArm(Arm.ArmAngle.NEUTRAL, TRANSFER.setNeutralArmWait)
                        ),
                        setArmstendo(Arm.Extension.RETRACTED, TRANSFER.retractArmstendoWait),
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

    // TODO
    public static Action setupBackWallPickup() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.BACK_WALL_PICKUP,
                new SequentialAction(
                        setLift(Lift.Ticks.BACK_WALL_PICKUP, BACK_WALL_PICKUP.setLiftToPickupFromBackWait),
                        new ParallelAction(
                                setArm(Arm.ArmAngle.BACK_WALL_PICKUP, BACK_WALL_PICKUP.setArmToPickupFromBackWait),
                                setClaw(Claw.ClawAngles.WALL_PICKUP, 0),
                                setWrist(Arm.WristAngle.BACK_WALL_PICKUP, 0)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.BACK_WALL_PICKUP)
                )
        );
    }

    // TODO
    public static Action setupBackWallSpecimen() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_BACK_SPECIMEN_FROM_WALL,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.CLAMPED, SETUP_SPECIMEN_WITH_ARMSTENDO.clampClawToPickupFromBackWait),
                        setLift(Lift.Ticks.SETUP_SPECIMEN_WITH_ARMSTENDO, SETUP_SPECIMEN_WITH_ARMSTENDO.extendLiftBeforeBackWallSpecimenWait),
                        new ParallelAction(
                                setArm(Arm.ArmAngle.SETUP_SPECIMEN_WITH_ARMSTENDO, 0),
                                setWrist(Arm.WristAngle.SETUP_SPECIMEN_WITH_ARMSTENDO, 0)
                        ),
                        setArmstendo(Arm.Extension.WALl_PICKUP, SETUP_SPECIMEN_WITH_ARMSTENDO.extendArmstendoBeforeBackWallSpecimenWait),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_BACK_SPECIMEN_FROM_WALL)
                )
        );
    }

    // TODO
    public static Action scoreBackWallSpecimen() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SCORE_BACK_WALL_SPECIMEN,
                new SequentialAction(
                        setArmstendo(Arm.Extension.EXTENDED, SCORE_SPECIMEN_WITH_ARMSTENDO.extendArmstendoToScoreSpecimenWait),
                        setClaw(Claw.ClawAngles.DEPOSIT, SCORE_SPECIMEN_WITH_ARMSTENDO.unclampClawToScoreSpecimenWait),
                        new InstantAction(() -> robot.currentState = Robot.State.SCORE_BACK_WALL_SPECIMEN)
                )
        );
    }

    // TODO
    public static Action stableTakeFromBackWallPickup(double sleepSecondsBeforeSetup) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_BACK_SPECIMEN_FROM_WALL,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.CLAMPED, STABLE_TAKE_FROM_BACK_WALL_PICKUP.clampClawToTakeSpecimenWait),
                        setLift(Lift.Ticks.BEFORE_BACK_SPECIMEN, sleepSecondsBeforeSetup),
                        new InstantAction(() -> robot.currentState = Robot.State.BACK_WALL_PICKUP),
                        setupBackWallSpecimen()
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


