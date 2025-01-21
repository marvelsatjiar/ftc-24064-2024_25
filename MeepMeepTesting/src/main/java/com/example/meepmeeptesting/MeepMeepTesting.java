package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static boolean
                is5plus0 = true,
                usePartnerSpec = false;



    public static double
            intakeSampleVelocityConstraint = 160,
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -29.5,
            sampleX = 21,
            sampleY = -44.6,
            secondSpecimenOffsetY = 3,
            thirdSpecimenOffsetY = 2.5,
            fourthSpecimenOffsetY = 3,
            fifthSpecimenOffsetY = 4,
            secondSpecimenOffsetX = -8,
            thirdSpecimenOffsetX = -5.5,
            fourthSpecimenOffsetX = -3,
            fifthSpecimenOffsetX = 3.5,
            sample1X = 47,
            sample2X = 55,
            sample3X = 62,
            startFirstSampleY = -12,
            startSampleY = -14,
            bumpSpecimen = -61,
            bumpSecondSpecimen = -62,
            intakeSpecimenY = -56,
            giveSample1X = sample1X - 4,
            giveSample2X = sample2X - 4,
            giveSample3X = sample3X,
            giveSampleY = -50,
            wallPickupX = 35,
            firstWallPickupX = 55,
            givingSampleAngle = 270,
            bumpSpecimenVelocityConstraint = 15,
            scoreSpecimenVelocityConstraint = 160,
            specimenAutoBasketX = -48,
            specimenAutoBasketY = -63,

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
        startPose = new Pose2d(startingPositionX,startingPositionY, Math.toRadians(270));

        Pose2d startSamplePose;
        startSamplePose = new Pose2d(-31.85,-63.375, Math.toRadians(0));


        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startPose);
        builder = scoreFirstSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);
        builder = scoreSample(builder);
        builder = park(builder);

//        builder = scoreSamples(builder);


        drive.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }

    private static TrajectoryActionBuilder scoreSample(TrajectoryActionBuilder builder) {
        builder = builder
                .strafeToSplineHeading(new Vector2d(sampleX, sampleY), Math.toRadians(315), (pose2dDual, posePath, v) -> intakeSampleVelocityConstraint)
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(specimenAutoBasketX, specimenAutoBasketY, Math.toRadians(0)), Math.toRadians(-135));
        return builder;
    }

    private static TrajectoryActionBuilder scoreSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xBasket, yBasket, Math.toRadians(45)), Math.toRadians(-135))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-55.0, -37.6, Math.toRadians(60)), Math.toRadians(45))
                .lineToY(bumpSample)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(xBasket, yBasket, Math.toRadians(45)), Math.toRadians(-45))
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(xSample2, ySample2, Math.toRadians(robotAngle)),Math.toRadians(110))
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(new Pose2d(xBasket, yBasket, Math.toRadians(45)),Math.toRadians(290))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xSample3, ySample3, Math.toRadians(thirdSampleangle)), Math.toRadians(thirdSampleangle))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(xSample3 - 2, ySample3 + 2), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(xBasket, yBasket, Math.toRadians(45)), Math.toRadians(255))         ;
        // Basket
        return builder;
    }

    private static TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
//                .setTangent(Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(12.0, -61.1), Math.toRadians(0));
        return builder;
    }

    public static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark) {
        builder = builder
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5 + offsetX, scoreSpecimenY + offsetY), Math.toRadians(90));
//                .strafeToConstantHeading(new Vector2d(5 + offsetX, scoreSpecimenY))
//                .stopAndAdd(RobotActions.scoreSpecimenFromFrontWallPickup());

        if (!doPark) {
//            builder = builder.afterTime(setupFrontWallPickupWait, RobotActions.setupFrontWallPickup());
            builder = builder
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(wallPickupX, intakeSpecimenY), Math.toRadians(270));
        }


//                .strafeToConstantHeading(new Vector2d(wallPickupX, intakeSpecimenY));

        if (!doPark) {
//            builder = builder.afterTime(startBumpToClampTime, RobotActions.takeSpecimenFromFrontWallPickup(true));
            builder = builder.lineToY(bumpSpecimen, (pose2dDual, posePath, v) -> bumpSpecimenVelocityConstraint);
        }


        return builder;

    }

    private static TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {
        builder = builder
                .splineToSplineHeading(new Pose2d(firstWallPickupX, intakeSpecimenY, Math.toRadians(270)), Math.toRadians(270))
//                .afterTime(startBumpToClampTime, RobotActions.takeSpecimenFromFrontWallPickup(true))
                .splineToSplineHeading(new Pose2d(firstWallPickupX, bumpSecondSpecimen, Math.toRadians(270)), Math.toRadians(270), (pose2dDual, posePath, v) -> bumpSpecimenVelocityConstraint);

        builder = scoreSpecimen(builder, 0, secondSpecimenOffsetY, false);
        builder = scoreSpecimen(builder, -2.5, 2, false);
        builder = scoreSpecimen(builder, -5, 3.75, !is5plus0);
        if (is5plus0)
            builder = scoreSpecimen(builder, -7.5,fifthSpecimenOffsetY, true);

        return builder;
    }
    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        boolean do3rdSample = is5plus0 || !usePartnerSpec;
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(35,-35),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(sample1X, startFirstSampleY), Math.toRadians(270))//, giveSampleVelConstraint)
                .splineToLinearHeading(new Pose2d(giveSample1X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(120))
//                .splineToLinearHeading(new Pose2d(46, -18, Math.toRadians(givingSampleAngle)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(sample2X, startSampleY), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d((!do3rdSample ? 4 : 0) + giveSample2X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(!do3rdSample ? 270 : 120));
        if (do3rdSample)
            builder = builder
                    .splineToConstantHeading(new Vector2d(sample3X, startSampleY), Math.toRadians(270))
                    .splineToLinearHeading(new Pose2d(giveSample3X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270));
        return builder;
    }

    private static TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToY(scoreSpecimenY);
        return builder;
    }


}