package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement.SubmersibleColorDetection;

public class SubDetectionTest extends AbstractAuto{
    private SubmersibleColorDetection submersibleColorDetection;

    public static double
            parkVelocityConstraint = 160,
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -30.5,
            parkX = 23,
            parkY = -44.6,
            extendSleep = 0.2,
            secondSpecimenOffsetY = 2,
            thirdSpecimenOffsetY = 2.5,
            fourthSpecimenOffsetY = 2.5,
            fifthSpecimenOffsetY = 2.5,
            secondSpecimenOffsetX = -11,
            thirdSpecimenOffsetX = -9,
            fourthSpecimenOffsetX = -5.5,
            fifthSpecimenOffsetX = -1,
            sample1X = 47,
            sample2X = 54.5,
            sample3X = 63,
            startFirstSampleY = -12,
            startSampleY = -14,
            giveSample2Y = -44.5,
            bumpSpecimen = -62.5,
            bumpSecondSpecimen = -62,
            intakeSpecimenY = -56,
            giveSample1X = sample1X - 4,
            giveSample2X = sample2X - 3,
            giveSample3X = sample3X,
            giveSampleY = -46,
            giveSample3Y = -45,
            wallPickupX = 41.5,
            firstWallPickupX = 58,
            secondSweeperSleep = 0.7,
            thirdSweeperSleep = 0.4,
            startBumpToClampTime = 0.4,
            secondSpecimenStartBumpToClampTime = 0.2,
            givingSampleAngle = 270,
            setupFrontWallPickupWait = 0.2,
            scoreSpecimenVelocityConstraint = 140,
            giveSampleVelocityConstraint = 27,
            scoreFirstSpecimenVelocityConstraint = 140,
            giveSecondSampleSweeperWait = 0.7,
            giveFirstSampleSweeperWait = 0.4,
            minProfileAccel = -30,
            maxProfileAccel = 60,
            minScoreProfileAccel = -50,
            maxScoreProfileAccel = 60,
            firstSpecimenWait = 0,
            minFirstProfileAccel = -45,
            clampWaitBeforeOverhangSpecimen = 0.2,
            retractAfterOverhangSpecimenWait = 0.6,
            setupOverhangSpecimenWait = 0.5,
            scoreToRetractWait = 0.7,
            sleepSecondsBeforeLimelightActivation = 0.5,
            sleepSecondsBeforeSubDetection = 0.8,
            sleepSecondsBeforeUnclampFirst = 1.2,
            sleepSecondsBeforeUnclampSecond = 2.3,
            sleepSecondsBeforeUnclampThird = 2.1,
            sleepSecondsBeforeUnclampFourth = 2,
            sleepSecondsBeforeUnclampFifth = 2,

    secondSpecimenSleepBeforeSetup = 0.5,
            lastThreeSleepBeforeSetup = 0.4,
            bumpSpecimenVelConstraint = 20;


    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(startingPositionX, startingPositionY, Math.toRadians(90));
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

        builder = scoreFirstSpecimen(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(sleepSecondsBeforeLimelightActivation, new ParallelAction(
                        new InstantAction(() -> submersibleColorDetection.activateLimelight()),
                        RobotActions.extendIntake(Extendo.Extension.THREE_FOURTHS)
                ))
                .afterTime(sleepSecondsBeforeSubDetection, new Actions.SingleCheckAction(
                        () -> submersibleColorDetection.lockSampleCounter != 9,
                        new InstantAction(() -> submersibleColorDetection.isSampleLocked = submersibleColorDetection.lockTargetSample())
                ))
                .lineToY((scoreSpecimenY), (pose2dDual, posePath, v) -> scoreFirstSpecimenVelocityConstraint, new ProfileAccelConstraint(minFirstProfileAccel, maxProfileAccel));

        if (submersibleColorDetection.isSampleLocked) builder = builder.stopAndAdd(submersibleColorDetection.driveToTarget());

        return builder;
    }

}
