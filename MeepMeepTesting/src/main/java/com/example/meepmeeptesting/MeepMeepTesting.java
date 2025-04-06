package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
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

    public static class GiveSamples {
        public double
                // Position
                intermediaryX = 34,
                intermediaryY = -33,

        intakeSampleX = 52,
                intakeSampleY = -41.5,

        outtakeSampleX = 52,
                outtakeSampleY = -41.5,
                sample1X = 47,
                sample1Y = -16,
                sample2X = 55,
                sample2Y = -14,
                sample3X = 63,
                sample3Y = -14,
                giveSampleY = -46,

        // Constraints
        giveSampleVelocityConstraint = 160,
                giveSampleMinAccelConstraint = -152.5,
                giveSampleMaxAccelConstraint = 150,

        // Timings
        sleepBeforeInterleaveSample = 0.5,
                firstIntakeDelay = 0.7,
                secondIntakeDelay = 0.7,
                thirdIntakeDelay = 0.5,
                firstSleepBeforeTurning = 0,
                secondSleepBeforeTurning = 0.5,
                thirdSleepBeforeTurning = 0.5,
                outtakeFirstSampleDelay = 0.7,
                outtakeSecondSampleDelay = 0.5,
                outtakeThirdSampleDelay = 0.4,
                sleepBeforeV4B = 0.3,
                thirdSleepBeforeV4B = 1,
                stopRollerDelay = 0.7,

        // Roller Power
        intakeRollerPower = 1,
                outtakeRollerPower = -1;
    }
    public static class ScoreSpecimens {
        public double
                // Positions
                subSampleX = 5,
                scoreSpecimenY = -27.5,

        wallPickupX = 33.5,
                secondWallPickupX = 52,
                intakeSpecimenY = -56.5,
                intakeSecondSpecimenY = -47,
                intakeSecondBumpSpecimenY = -64.5,
                specimen2ndOffsetX = -20,
                specimen3rdOffsetX = -18,
                specimen4thOffsetX = -16,
                specimen5thOffsetX = -14,
                specimen6thOffsetX = -13,
                specimen7thOffsetX = -11,
                secondSpecimenOffsetY = 7,
                thirdSpecimenOffsetY = 7,
                fourthSpecimenOffsetY = 7,
                fifthSpecimenOffsetY = 7,
                sixthSpecimenOffsetY = 7,
                seventhSpecimenOffsetY = 7,

        // Headings
        scoringAngle = 105,

        // Timings
        waitUntilExtendoRetracted = 0.4,
                retractAfterIntakeSub = 0,
                secondsToExpire = 1.5,
                timeBeforeWallPickup = 1,
                timeBeforeWrist = 0.2,
                timeBeforeMoving = 0.1,
                lastFourSleepBeforeGrab = 0,
                secondSleepBeforeSetup = 0.1,
                sleepSecondsBeforeUnclampFirst = 1.4,
                sleepSecondsBeforeUnclampSecond = 2,
                sleepSecondsBeforeUnclampThird = 1.5,
                sleepSecondsBeforeUnclampFourth = 1.5,
                sleepSecondsBeforeUnclampFifth = 1.5,
                sleepSecondsBeforeUnclampSixth = 1.3,
                sleepSecondsBeforeUnclampSeventh = 1.5,
                timeBeforeInterleave = 0.3,
                v4bWait = 0.5,
                sleepBeforeTransfer = 1,


        //Constraints
        scoreFirstSpecimenVelocityConstraint = 160,
                maxFirstProfileAccel = -150,
                minFirstProfileAccel = 150,
                wallPickUpVelocityConstraint = 60,
                scoreSpecimenVelocityConstraint = 160,
                minScoreProfileAccel = -140,
                maxScoreProfileAccel = 150;
    }
    public static GiveSamples G_S = new GiveSamples();
    public static ScoreSpecimens S_S = new ScoreSpecimens();

    public static double
            startingPositionX = 7.375,
            startingPositionY = -60,
            parkX = 23,
            parkY = -44.6,
            extendSleep = 0.2;

//

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
                .setConstraints(50, 60, 5, 10, 11.75)
                .setDimensions(14,  16.5)
                .build();

        Pose2d startPose;
        startPose = new Pose2d(startingPositionX, startingPositionY, Math.toRadians(90));

        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startPose);
        builder = scoreFirstSpecimen(builder);
        builder = scoreSixthAndSeventhSpecimen(builder);
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
    private TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
//                .afterTime(extendSleep, new ParallelAction(
//                        RobotActions.setExtendo(Extendo.Extension.EXTENDED,0),
//                        RobotActions.setArm(Arm.ArmAngle.BASKET,0),
//                        RobotActions.setWrist(Arm.WristAngle.BASKET,0),
//                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
//                ))
                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(315));
        return builder;
    }

    private static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark, double sleepSecondsBeforeUnclamp, boolean doVision) {
        // Scoring
        builder = builder
                .setTangent(90);
//                .afterTime(sleepSecondsBeforeUnclamp, RobotActions.scoreSpecimen());

//        if (doVision) {
//            builder = builder
//                    .afterTime(
//                            sleepSecondsBeforeUnclamp,
//                            new SequentialAction(
//                                    autoAlignToSample.detectTarget(S_S.secondsToExpire, true),
//                                    new InstantAction(() -> robot.limelightEx.enableStagelite(false))
//                            )
//                    );
//        }

        builder = builder.strafeToLinearHeading(new Vector2d(10 + offsetX, S_S.scoreSpecimenY + offsetY), Math.toRadians(S_S.scoringAngle), (pose2dDual, posePath, v) -> S_S.scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(S_S.minScoreProfileAccel, S_S.maxScoreProfileAccel));

//        if (doVision) {
//            builder = builder
//                    .stopAndAdd(new SequentialAction(
//                            new InstantAction(autoAlignToSample::generateTargetTrajectory),
//                            telemetryPacket -> {
//                                robot.run();
//                                return autoAlignToSample.getTargetSampleTrajectory().run(telemetryPacket);
//                            }
//                    ));
//        }


//        if (doVision) {
//            builder = builder
//                    .afterTime(G_S.sleepBeforeInterleaveSample, new SequentialAction(
//                            RobotActions.transfer(),
//                            RobotActions.interleaveDropSample())
//                    );
//        } else if (!doPark) builder = builder.afterTime(S_S.timeBeforeWallPickup, RobotActions.setupWallPickup());


        builder = builder
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(S_S.wallPickupX, S_S.intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270), (pose2dDual, posePath, v) -> S_S.wallPickUpVelocityConstraint);

        // Setting up for the next cycle
//        if (!doPark) builder = builder
//                .stopAndAdd(new SequentialAction(
//                        RobotActions.setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, S_S.timeBeforeWrist),
//                        RobotActions.setWrist(Arm.WristAngle.GRAB_OFF_WALL, S_S.timeBeforeMoving)
//                ))
//                .afterTime(S_S.lastFourSleepBeforeGrab, RobotActions.setupSpecimen());


        return builder;
    }

    private static TrajectoryActionBuilder     scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .setTangent(Math.toRadians(270))
                .lineToY(S_S.intakeSecondBumpSpecimenY);
//                .stopAndAdd(new SequentialAction(
//                        RobotActions.setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, S_S.timeBeforeWrist),
//                        RobotActions.setWrist(Arm.WristAngle.GRAB_OFF_WALL, S_S.timeBeforeMoving)
//                ))
//                .afterTime(S_S.secondSleepBeforeSetup, RobotActions.setupSpecimen());

        builder = scoreSpecimen(builder, S_S.specimen2ndOffsetX, S_S.secondSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampSecond, false);
        builder = scoreSpecimen(builder, S_S.specimen3rdOffsetX, S_S.thirdSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampThird, true);
        builder = scoreSpecimen(builder, S_S.specimen4thOffsetX, S_S.fourthSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampFourth, false);
        builder = scoreSpecimen(builder, S_S.specimen5thOffsetX, S_S.fifthSpecimenOffsetY, true, S_S.sleepSecondsBeforeUnclampFifth, false);

        return builder;
    }

    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(G_S.intermediaryX,G_S.intermediaryY), Math.toRadians(90), (pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint))
                .splineToConstantHeading(new Vector2d(G_S.sample1X, G_S.sample1Y), Math.toRadians(270), (pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint))
                .lineToY(G_S.giveSampleY, (pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(G_S.sample2X, G_S.sample2Y, Math.toRadians(90)), Math.toRadians(45),(pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint))
                .setTangent(Math.toRadians(270))
                .lineToY(G_S.giveSampleY, (pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(G_S.sample3X, G_S.sample3Y, Math.toRadians(90)), Math.toRadians(45), (pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint))
                .setTangent(Math.toRadians(270))
                .lineToY(G_S.giveSampleY, (pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint));

        return builder;
    }

    private static TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
//                .afterTime(0, RobotActions.setupSpecimen())
//                .afterTime(0, new InstantAction(() -> autoAlignToSample.activateLimelight(IS_RED ? LIMELIGHT_RED_DETECTION_PIPELINE : LIMELIGHT_BLUE_DETECTION_PIPELINE, IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE)))
//                .afterTime(S_S.sleepSecondsBeforeUnclampFirst, RobotActions.scoreSpecimen())
                .strafeToConstantHeading(new Vector2d(S_S.subSampleX, S_S.scoreSpecimenY), (pose2dDual, posePath, v) -> S_S.scoreFirstSpecimenVelocityConstraint);

//        if (isSevenPlusZero) {
//            builder = builder
//                    .stopAndAdd(
//                            new SequentialAction(
//                                    autoAlignToSample.detectTarget(S_S.secondsToExpire, true),
//                                    new InstantAction(() -> robot.limelightEx.enableStagelite(false))
//                            ))
//                    .stopAndAdd(new SequentialAction(
//                            new InstantAction(autoAlignToSample::generateTargetTrajectory),
//                            telemetryPacket -> {
//                                robot.run();
//                                return autoAlignToSample.getTargetSampleTrajectory().run(telemetryPacket);
//                            }
//                    ));
//        }

//        if (isSixPlusZero) builder = builder
//                .stopAndAdd(new SequentialAction(
//                        RobotActions.setExtendo(S_S.intakeSubSampleExtendoAngle, S_S.v4bWait),
//                        new ParallelAction(
//                                RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
//                                RobotActions.setRollers(1, S_S.sleepBeforeTransfer)),
//                        RobotActions.transfer()
//                ))
//                .afterTime(S_S.timeBeforeInterleave, RobotActions.interleaveDropSample());

        return builder;
    }

    private static TrajectoryActionBuilder scoreSixthAndSeventhSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(315))
//                .afterTime(G_S.sleepBeforeInterleaveSample, new SequentialAction(
//                        RobotActions.transfer(),
//                        RobotActions.interleaveDropSample())
//                )
                .splineToLinearHeading(new Pose2d(S_S.wallPickupX, S_S.intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270), (pose2dDual, posePath, v) -> S_S.wallPickUpVelocityConstraint)//                .stopAndAdd(new SequentialAction(
//                        RobotActions.setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, S_S.timeBeforeWrist),
//                        RobotActions.setWrist(Arm.WristAngle.GRAB_OFF_WALL, S_S.timeBeforeMoving)
//                ))
//                .afterTime(S_S.secondSleepBeforeSetup, RobotActions.setupSpecimen());

//                .afterTime(sleepSecondsBeforeUnclamp, RobotActions.scoreSpecimen());
                .strafeToLinearHeading(new Vector2d(10 + S_S.specimen5thOffsetX, S_S.scoreSpecimenY + S_S.sixthSpecimenOffsetY), Math.toRadians(90), (pose2dDual, posePath, v) -> S_S.scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(S_S.minScoreProfileAccel, S_S.maxScoreProfileAccel));

        return builder;
    }
}