package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static boolean
            is5plus0 = true,
            usePartnerSpec = false,
            isSixPlusZero = true;
    public static class GiveSamples {
        public double
                // Position
                outtakeVelocityConstraint = 60,
                intermediaryX = 32.5,
                intermediaryY = -51,

        intakeSampleX = 52,
                intakeSampleY = -41.5,

        outtakeSampleX = 52,
                outtakeSampleY = -41.5,


        // Headings
        tangentBeforeFirstSample = 0,
                tangentForIntermediaryPosition = 0,

        intakeFirstSampleHeading = 90,
                intakeSecondSampleHeading = 54,
                intakeThirdSampleHeading = 39,
                outtakeFirstSampleHeading = -70,
                outtakeSecondSampleHeading = -70,
                outtakeThirdSampleHeading = -70,
                intakeFirstExtendoAngle = 80,
                intermediarySecondExtendoAngle = 52,
                intakeSecondExtendoAngle = 90,
                intakeThirdExtendoAngle = 120,
                outtakeExtendoAngleFirst = 70,
                outtakeExtendoAngleSecond = 80,
                outtakeExtendoAngleThird = 80,

        // Timings
        sleepBeforeInterleaveSample = 0.5,
                firstIntakeDelay = 0.7,
                secondIntakeDelay = 0,
                secondIntermediaryDelay = 0.2,
                thirdIntakeDelay = 0.5,
                firstSleepBeforeTurning = 0.3,
                secondSleepBeforeTurning = 0.5,
                thirdSleepBeforeTurning = 0.5,
                outtakeFirstSampleDelay = 0.6,
                outtakeSecondSampleDelay = 0.5,
                outtakeThirdSampleDelay = 0.4,
                sleepBeforeV4B = 0,
                thirdSleepBeforeV4B = 1,
                stopRollerDelay = 0.7,

        // Roller Power
        intakeRollerPower = 1,
                outtakeRollerPower = -1;
    }



    public static double
            outtakeVelocityConstraint = 60,
            parkVelocityConstraint = 160,
            startingPositionX = 7.375,
            startingPositionY = -60,
            scoreSpecimenY = -37.5,
            intermediaryX = 32.5,
            intermediaryY = -51,
            waitBeforeOuttakeSample = 0.4,
            waitToExtendTo2ndSample = 0.4,
            setBackWallPickupWait = 0.8,
            outtakeSampleDelay = 0.3,
            intakeSampleDelay = 0.5,
            getSampleX = 37.5,
            firstSampleX = 46,
            secondSampleX = 45,
            thirdSampleX = 45,
            giveFirstSampleX = 41,
            giveSecondSampleX = 44,
            giveThirdSampleX = 46,
            intakeSampleY = -40,
            outtakeSampleY = -44,
            outtakeSampleHeading = 330,
            intakeSampleHeading = 45,
            parkX = 23,
            parkY = -44.6,
            extendSleep = 0.2,
            secondSpecimenOffsetY = 11,
            thirdSpecimenOffsetY = 11,
            fourthSpecimenOffsetY = 11,
            fifthSpecimenOffsetY = 11,
            sixthSpecimenOffsetY = 11,
            secondSpecimenOffsetX = -11,
            thirdSpecimenOffsetX = -9,
            fourthSpecimenOffsetX = -16,
            fifthSpecimenOffsetX = -1,
            sixthSpecimenOffsetX = 0.5,
            sleepSecondsBeforeSetupSecond = 0.8,
            sleepSecondsBeforeSetupThird = 0.6,
            sleepSecondsBeforeSetupFourth = 0.7,
            sleepSecondsBeforeSetupFifth = 0.8,
            sleepSecondsBeforeSetupSixth = 0.9,
            bumpSpecimen = -62.5,
            bumpSecondSpecimen = -62,
            pickupSecondSpecimenX = 40,
            intakeSpecimenY = -62.5,
            dropOffX = 40,
            dropOffY = -60.5,
            dropOffHeading = 315,
            wallPickupX = 45.5,
            startBumpToClampTime = 0.4,
            intakeSpecimenVelocityConstraint = 90,
            scoreSpecimenVelocityConstraint = 140,
            scoreFirstSpecimenVelocityConstraint = 140,
            maxProfileAccel = 60,
            minScoreProfileAccel = -50,
            maxScoreProfileAccel = 60,
            minFirstProfileAccel = -45,
            subSampleX = 0,
            scoreToRetractWait = 0.3,
            sleepSecondsBeforeLimelightActivation = 0.5,
            sleepSecondsBeforeSubDetection = 0.8,
            sleepSecondsBeforeUnclampFirst = 1.2,
            sleepSecondsBeforeUnclampSecond = 2,
            sleepSecondsBeforeUnclampThird = 2,
            sleepSecondsBeforeUnclampFourth = 2,
            sleepSecondsBeforeUnclampFifth = 2,
            sleepSecondsBeforeUnclampSixth = 2,
            secondSpecimenSleepBeforeSetup = 0.5,
            bumpSpecimenVelConstraint = 20;
//

//    private static final VelConstraint giveSampleVelConstraint = (robotPose, path, disp) -> {
//        if (robotPose.position.y.value() > -17) {
//            return slowDownConstraint;
//        } else {
//            return 60.0;
//        }
//    };
    public static GiveSamples G_S = new GiveSamples();


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        boolean isSpecimenSide = true;

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, 5, 10, 11.75)
                .setDimensions(14,  16.5)
                .build();

        Pose2d startPose;
        startPose = new Pose2d(startingPositionX,startingPositionY, Math.toRadians(90));

        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startPose);
        builder = scoreFirstSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);
//        builder = park(builder);


        drive.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }

//    private TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
//        builder = builder
////                .afterTime(extendSleep, new ParallelAction(
////                        RobotActions.setExtendo(Extendo.Extension.EXTENDED,0),
////                        RobotActions.setArm(Arm.ArmAngle.BASKET,0),
////                        RobotActions.setWrist(Arm.WristAngle.BASKET,0),
////                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
////                ))
//                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(315));
//        return builder;
//    }

    private static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark, double sleepSecondsBeforeSetup, double sleepSecondsBeforeUnclamp, boolean doVision) {
        // Scoring
        builder = builder
                .setTangent(90)
                .strafeToLinearHeading(new Vector2d(10 + offsetX, scoreSpecimenY + offsetY), Math.toRadians(90));//, (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(minScoreProfileAccel, maxScoreProfileAccel))

        if (doVision)
            builder = builder
                .setTangent(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(dropOffX, dropOffY), Math.toRadians(dropOffHeading))
                .splineToSplineHeading(new Pose2d(wallPickupX, intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270));
        else
            builder = builder
                    .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(wallPickupX, dropOffY, Math.toRadians(90)), Math.toRadians(315))
                    .lineToY(intakeSpecimenY);


        // Setting up for the next cycle
        if (!doPark) {
            builder = builder
//                    .afterTime(startBumpToClampTime, RobotActions.setupSpecimen())
                    .lineToY(bumpSpecimen); // ((pose2dDual, posePath, v) -> bumpSpecimenVelConstraint)
        }

        return builder;
    }

    private static TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .setTangent(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(52, intakeSpecimenY), Math.toRadians(90));

        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, secondSpecimenOffsetY, false, sleepSecondsBeforeSetupSecond, sleepSecondsBeforeUnclampSecond, false);
        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, thirdSpecimenOffsetY, false, sleepSecondsBeforeSetupThird, sleepSecondsBeforeUnclampThird, false);
        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, fourthSpecimenOffsetY, false, sleepSecondsBeforeSetupFourth, sleepSecondsBeforeUnclampFourth, false);
        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, fifthSpecimenOffsetY, !isSixPlusZero, sleepSecondsBeforeSetupFifth, sleepSecondsBeforeUnclampFifth, false);

        if (isSixPlusZero) builder = scoreSpecimen(builder, fourthSpecimenOffsetX, sixthSpecimenOffsetY, true, sleepSecondsBeforeSetupSixth, sleepSecondsBeforeUnclampSixth, false);

        return builder;
    }
    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(325))
                .splineToSplineHeading(new Pose2d(G_S.intermediaryX, G_S.intermediaryY, Math.toRadians(90)), Math.toRadians(G_S.tangentForIntermediaryPosition))

                // Intaking 1st

                .setTangent(Math.toRadians(G_S.tangentBeforeFirstSample))
                .splineToSplineHeading(new Pose2d(G_S.intakeSampleX, G_S.intakeSampleY, Math.toRadians(G_S.intakeFirstSampleHeading)), Math.toRadians(35))
                .waitSeconds(G_S.firstSleepBeforeTurning)

                // Outtaking 1st

                //.afterTime(G_S.setV4bDownWhenFirstSampleOuttakeDelay, RobotActions.setV4B(Intake.V4BAngle.DOWN, 0))
                .strafeToLinearHeading(new Vector2d(G_S.outtakeSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeFirstSampleHeading), (pose2dDual, posePath, v) -> G_S.outtakeVelocityConstraint)

                // Intaking 2nd

                .strafeToLinearHeading(new Vector2d(G_S.intakeSampleX, G_S.intakeSampleY), Math.toRadians(G_S.intakeSecondSampleHeading))
                .waitSeconds(G_S.secondSleepBeforeTurning)

                // Outtaking 2nd


                .strafeToLinearHeading(new Vector2d(G_S.outtakeSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeSecondSampleHeading), (pose2dDual, posePath, v) -> G_S.outtakeVelocityConstraint)

                //Intaking 3rd

                .strafeToLinearHeading(new Vector2d(G_S.intakeSampleX, G_S.intakeSampleY), Math.toRadians(G_S.intakeThirdSampleHeading))
                .waitSeconds(G_S.thirdSleepBeforeTurning)
                //Outtaking 3rd

                .strafeToLinearHeading(new Vector2d(G_S.outtakeSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeThirdSampleHeading), (pose2dDual, posePath, v) -> G_S.outtakeVelocityConstraint);

        return builder;
    }

    private static TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .strafeToConstantHeading(new Vector2d(subSampleX, scoreSpecimenY));//, (pose2dDual, posePath, v) -> scoreFirstSpecimenVelocityConstraint, new ProfileAccelConstraint(minFirstProfileAccel, maxProfileAccel));

        return builder;
    }


}