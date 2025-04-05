package com.example.meepmeeptesting;

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
                intermediaryX = 22,
                intermediaryY = -39,

                intakeSampleX = 52,
                intakeSampleY = -41.5,

                outtakeSampleX = 52,
                outtakeSampleY = -41.5,

                // Constraints
                outtakeSampleVelocityConstraint = 120,
                outtakeSampleMinAccelConstraint = -95,
                outtakeSampleMaxAccelConstraint = 110,

                // Headings
                tangentBeforeFirstSample = 0,
                tangentForIntermediaryPosition = 0,

                intakeFirstSampleHeading = 91,
                intakeSecondSampleHeading = 65,
                intakeThirdSampleHeading = 46,
                outtakeFirstSampleHeading = -70,
                outtakeSecondSampleHeading = -70,
                outtakeThirdSampleHeading = -70,
                intakeFirstExtendoAngle = 80,
                intakeSecondExtendoAngle = 72,
                intakeThirdExtendoAngle = 130,
                outtakeExtendoAngleFirst = 20,
                outtakeExtendoAngleSecond = 50,
                outtakeExtendoAngleThird = 50,

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
                secondWallPickupX = 63,
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
                giveSampleVelocityConstraint = 120,
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
//        builder = scoreSixthAndSeventhSpecimen(builder);
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

    private static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark, double sleepSecondsBeforeUnclamp) {
        // Scoring
        builder = builder
                .setTangent(90)
                .strafeToLinearHeading(new Vector2d(10 + offsetX, S_S.scoreSpecimenY + offsetY), Math.toRadians(100), (pose2dDual, posePath, v) -> 160, new ProfileAccelConstraint(-150, 150))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(S_S.wallPickupX, S_S.intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270), (pose2dDual, posePath, v) -> 160, new ProfileAccelConstraint(-150, 150));

        return builder;
    }

    private static TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .setTangent(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(S_S.secondWallPickupX, S_S.intakeSecondSpecimenY), Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .lineToY(S_S.intakeSecondBumpSpecimenY);

        builder = scoreSpecimen(builder, S_S.specimen2ndOffsetX, S_S.secondSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampSecond);
        builder = scoreSpecimen(builder, S_S.specimen3rdOffsetX, S_S.thirdSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampThird);
        builder = scoreSpecimen(builder, S_S.specimen4thOffsetX, S_S.fourthSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampFourth);
        builder = scoreSpecimen(builder, S_S.specimen5thOffsetX, S_S.fifthSpecimenOffsetY, true, S_S.sleepSecondsBeforeUnclampFifth);

        if (isSixPlusZero) builder = scoreSpecimen(builder, S_S.specimen4thOffsetX, S_S.sixthSpecimenOffsetY, true, S_S.sleepSecondsBeforeUnclampFifth);

        return builder;
    }
    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        boolean do3rdSample = is5plus0 || !usePartnerSpec;
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(34,-33), Math.toRadians(90), (pose2dDual, posePath, v) -> 160, new ProfileAccelConstraint(-150, 150))
                .splineToConstantHeading(new Vector2d(47, -16), Math.toRadians(270), (pose2dDual, posePath, v) -> 160, new ProfileAccelConstraint(-150, 150))
                .lineToY(-46, (pose2dDual, posePath, v) -> 160, new ProfileAccelConstraint(-150, 150))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, -14, Math.toRadians(90)), Math.toRadians(45),(pose2dDual, posePath, v) -> 160, new ProfileAccelConstraint(-150, 150))
                .setTangent(Math.toRadians(270))
                .lineToY(-46, (pose2dDual, posePath, v) -> 160, new ProfileAccelConstraint(-150, 150))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(63, -14, Math.toRadians(90)), Math.toRadians(45), (pose2dDual, posePath, v) -> 160, new ProfileAccelConstraint(-150, 150))
                .setTangent(Math.toRadians(270))
                .lineToY(-46, (pose2dDual, posePath, v) -> 160, new ProfileAccelConstraint(-150, 150))         ;

        return builder;
    }




    private static TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .strafeToConstantHeading(new Vector2d(S_S.subSampleX, S_S.scoreSpecimenY));//, (pose2dDual, posePath, v) -> scoreFirstSpecimenVelocityConstraint, new ProfileAccelConstraint(minFirstProfileAccel, maxProfileAccel));

        return builder;
    }

//    private static TrajectoryActionBuilder scoreSixthAndSeventhSpecimen(TrajectoryActionBuilder builder) {
//        builder = builder
//                .strafeToLinearHeading(new Vector2d(wallPickupX, intakeSpecimenY), Math.toRadians(90))
//                .lineToY(intakeSpecimenY);
//
//        builder = scoreSpecimen(builder, sixthSpecimenOffsetX, sixthSpecimenOffsetY, false, sleepSecondsBeforeSetupSixth, sleepSecondsBeforeUnclampSixth);
//
//        builder = builder
//                .setTangent(90)
//                .strafeToLinearHeading(new Vector2d(10 + seventhSpecimenOffsetX, scoreSpecimenY + seventhSpecimenOffsetY), Math.toRadians(100)); //, (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(minScoreProfileAccel, maxScoreProfileAccel))
//        return builder;
//    }

}