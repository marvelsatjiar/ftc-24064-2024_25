package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static boolean
                is5plus0 = true,
                usePartnerSpec = false,
                isSixPlusZero = false;



    public static double
            parkVelocityConstraint = 160,
            startingPositionX = 7.375,
            startingPositionY = -60,
            scoreSpecimenY = -33.5,
            intermediaryX = 25,
            waitBeforeOuttakeSample = 0.4,
            waitToExtendTo2ndSample = 0.4,
            setBackWallPickupWait = 0.8,
            outtakeSampleDelay = 0.3,
            intakeSampleDelay = 0.5,
            getSampleX = 37.5,
            firstSampleX = 30.5,
            secondSampleX = 43,
            thirdSampleX = 45,
            giveFirstSampleX = 41,
            giveSecondSampleX = 44,
            giveThirdSampleX = 46,
            intakeSampleY = -39,
            outtakeSampleY = -44,
            outtakeSampleHeading = 330,
            intakeSampleHeading = 45,
            parkX = 23,
            parkY = -44.6,
            extendSleep = 0.2,
            secondSpecimenOffsetY = 2,
            thirdSpecimenOffsetY = 2.5,
            fourthSpecimenOffsetY = 2.5,
            fifthSpecimenOffsetY = 2.5,
            sixthSpecimenOffsetY = 2.5,
            secondSpecimenOffsetX = -11,
            thirdSpecimenOffsetX = -9,
            fourthSpecimenOffsetX = -5.5,
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
            wallPickupX = 41.5,
            startBumpToClampTime = 0.4,
            intakeSpecimenVelocityConstraint = 90,
            scoreSpecimenVelocityConstraint = 140,
            scoreFirstSpecimenVelocityConstraint = 140,
            maxProfileAccel = 60,
            minScoreProfileAccel = -50,
            maxScoreProfileAccel = 60,
            minFirstProfileAccel = -45,
            subSampleX = 5,
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
            bumpSpecimenVelConstraint = 20,
//            Sample variables start here
            bumpSample = -36,
            startingSamplePositionX = 7.375,
            startingSamplePositionY = -62,
            scoreSampleY = -29,
            scoreSpecimenX = -4,
            waitToScoreSample1 = 4,
            waitToScoreSample2 = 4,
            waitToScoreSample3 = 4,
            robotAngle = 97,
            thirdSampleangle = 148,
            xSample1 = -45.6,
            ySample1 = -34,
            xSample2 = -57.5,
            ySample2 = -36,
            xSample3 = -46.75,
            ySample3 = -40,
            xBasket = -54.25,
            yBasket = -54.25;

//    private static final VelConstraint giveSampleVelConstraint = (robotPose, path, disp) -> {
//        if (robotPose.position.y.value() > -17) {
//            return slowDownConstraint;
//        } else {
//            return 60.0;
//        }
//    };

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        boolean isSpecimenSide = true;

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 11.75)
                .setDimensions(14,  16.5)
                .build();

        Pose2d startPose;
        startPose = new Pose2d(startingPositionX,startingPositionY, Math.toRadians(90));

        Pose2d startSamplePose;
        startSamplePose = new Pose2d(-31.85,-63.375, Math.toRadians(0));

        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startPose);
        builder = scoreFirstSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);
//        builder = park(builder);


//        builder = scoreSamples(builder);


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

    private static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark, double sleepSecondsBeforeSetup, double sleepSecondsBeforeUnclamp) {
        // Scoring
        builder = builder
                .setTangent(90)
//                .afterTime(sleepSecondsBeforeUnclamp, RobotActions.scoreSpecimen())
                .strafeToLinearHeading(new Vector2d(10 + offsetX, scoreSpecimenY + offsetY), Math.toRadians(100))//, (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(minScoreProfileAccel, maxScoreProfileAccel))
                .setTangent(Math.toRadians(315))
//                .afterTime(setBackWallPickupWait, RobotActions.setupWallPickup())
                .splineToLinearHeading(new Pose2d(wallPickupX, intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270));


        // Setting up for the next cycle
//        if (!doPark) {
//            builder = builder
////                    .afterTime(startBumpToClampTime, RobotActions.setupSpecimen())
//                    .lineToY(bumpSpecimen); // ((pose2dDual, posePath, v) -> bumpSpecimenVelConstraint)
//        }

        return builder;
    }

    private static TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(wallPickupX, intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270))
                .lineToY(bumpSpecimen);
//                .afterTime(startBumpToClampTime, RobotActions.setupSpecimen());

        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, secondSpecimenOffsetY, false, sleepSecondsBeforeSetupSecond, sleepSecondsBeforeUnclampSecond);
        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, thirdSpecimenOffsetY, false, sleepSecondsBeforeSetupThird, sleepSecondsBeforeUnclampThird);
        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, fourthSpecimenOffsetY, false, sleepSecondsBeforeSetupFourth, sleepSecondsBeforeUnclampFourth);
        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, fifthSpecimenOffsetY, !isSixPlusZero, sleepSecondsBeforeSetupFifth, sleepSecondsBeforeUnclampFifth);

        if (isSixPlusZero) builder = scoreSpecimen(builder, fourthSpecimenOffsetX, sixthSpecimenOffsetY, true, sleepSecondsBeforeSetupSixth, sleepSecondsBeforeUnclampSixth);

        return builder;
    }
    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
//                .setTangent(Math.toRadians(90))
//                .afterTime(waitBeforeOuttakeSample, RobotActions.extendIntake(Extendo.Extension.ONE_HALF))
                .splineToSplineHeading(new Pose2d(intermediaryX, intakeSampleY, Math.toRadians(90)), Math.toRadians(0))
////                .afterTime(0, new SequentialAction(
////                        RobotActions.setRollers(-1, outtakeSampleDelay),
////                        RobotActions.setRollers(0, 0)
////                ))
//
                .splineToSplineHeading(new Pose2d(firstSampleX, intakeSampleY, Math.toRadians(intakeSampleHeading)), Math.toRadians(0))
//                .afterTime(0, new SequentialAction(
//                        RobotActions.setRollers(1, intakeSampleDelay),
//                        RobotActions.setRollers(0, 0)
//                ))
                .strafeToLinearHeading(new Vector2d(giveFirstSampleX, outtakeSampleY), Math.toRadians(outtakeSampleHeading))
////                .afterTime(0, new SequentialAction(
////                        RobotActions.setRollers(-1, outtakeSampleDelay),
////                        RobotActions.setRollers(0, 0)
////                ))
//
////                .afterTime(waitToExtendTo2ndSample, RobotActions.extendIntake(Extendo.Extension.THREE_FOURTHS))
                .strafeToLinearHeading(new Vector2d(secondSampleX, intakeSampleY), Math.toRadians(intakeSampleHeading))
////                .afterTime(0, new SequentialAction(
////                        RobotActions.setRollers(1, intakeSampleDelay),
////                        RobotActions.setRollers(0, 0)
////                ))
                .strafeToLinearHeading(new Vector2d(giveSecondSampleX, outtakeSampleY), Math.toRadians(outtakeSampleHeading))
////                .afterTime(0, new SequentialAction(
////                        RobotActions.setRollers(-1, outtakeSampleDelay),
////                        RobotActions.setRollers(0, 0)
////                ))
//
//                .setTangent(180)
                .strafeToLinearHeading(new Vector2d(thirdSampleX, intakeSampleY), Math.toRadians(intakeSampleHeading))
////                .afterTime(0, new SequentialAction(
////                        RobotActions.setRollers(1, intakeSampleDelay),
////                        RobotActions.setRollers(0, 0)
////                ))
                .strafeToLinearHeading(new Vector2d(giveThirdSampleX, outtakeSampleY), Math.toRadians(outtakeSampleHeading));
////                .afterTime(0, new SequentialAction(
////                        RobotActions.setRollers(-1, outtakeSampleDelay),
////                        RobotActions.retractExtendo(),
////                        RobotActions.setRollers(0, 0),
////                        RobotActions.setupWallPickup()
////                ))

        return builder;
    }

    private static TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
//        if (isSixPlusZero) {
//            builder = builder
//                    .afterTime(sleepSecondsBeforeLimelightActivation, new ParallelAction(
//                            new InstantAction(() -> autoAlignToSample.activateLimelight()),
//                            RobotActions.extendIntake(Extendo.Extension.THREE_FOURTHS)
//                    ))
//                    .afterTime(sleepSecondsBeforeSubDetection, new Actions.SingleCheckAction(
//                            () -> autoAlignToSample.lockSampleCounter != 9,
//                            new InstantAction(() -> autoAlignToSample.isSampleLocked = autoAlignToSample.lockTargetSample())
//                    ));
//        }

        builder = builder
//                .afterTime(0, RobotActions.setupSpecimen())
//                .afterTime(sleepSecondsBeforeUnclampFirst, RobotActions.scoreSpecimen())
                .strafeToConstantHeading(new Vector2d(subSampleX, scoreSpecimenY));//, (pose2dDual, posePath, v) -> scoreFirstSpecimenVelocityConstraint, new ProfileAccelConstraint(minFirstProfileAccel, maxProfileAccel));

//        if (autoAlignToSample.isSampleLocked && isSixPlusZero) {
//            builder = builder
//                    .stopAndAdd(new SequentialAction(
//                            new ParallelAction(
//                                    RobotActions.setRollers(1, 0),
//                                    autoAlignToSample.driveToTarget()
//                            ),
//                            RobotActions.retractExtendo()
//                    ));
//        }

        return builder;
    }


}