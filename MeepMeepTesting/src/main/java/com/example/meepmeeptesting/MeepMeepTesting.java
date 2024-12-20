package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static boolean
                is5plus0 = true,
                usePartnerSpec = false;



    public static double
            slowDownConstraint = 20,
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -30,
            secondSpecimenOffsetY = 2,
            fifthSpecimenOffsetY = 4.75,
            sample1X = 47,
            sample2X = 55,
            sample3X = 62,
            startSampleY = -14,
            slowDownStartY = -25,
            bumpSpecimen = -60,
            bumpSecondSpecimen = -62,
            intakeSpecimenY = -56,
            giveSample1X = sample1X - 4,
            giveSample2X = sample2X - 4,
            giveSample3X = sample3X,
            giveSampleY = -50,
            wallPickupX = 35,
            firstWallPickupX = 55,
            sweeperSleep = 1,
            startBumpToClampTime = 0.35,
            givingSampleAngle = 270,
            regularGivingConstraint = 40,
            setupFrontWallPickupWait = 0.2,
            bumpSpecimenVelocityConstraint = 15,
            scoreSpecimenVelocityConstraint = 70;

    private static final VelConstraint giveSampleVelConstraint = (robotPose, path, disp) -> {
        if (robotPose.position.y.value() > -17) {
            return slowDownConstraint;
        } else {
            return 60.0;
        }
    };

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        boolean isSpecimenSide = true;

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 11.75)
                .setDimensions(14,  16.5)
                .build();

        Pose2d startPose;
        startPose = new Pose2d(7.375,-62,Math.toRadians(270));


        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startPose);
        builder = scoreFirstSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);
        builder = park(builder);

        drive.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }

    private static TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
//                .setTangent(Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(12.0, -41.1), Math.toRadians(315));
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
                .splineToConstantHeading(new Vector2d(sample1X, startSampleY), Math.toRadians(270))//, giveSampleVelConstraint)
                .splineToLinearHeading(new Pose2d(giveSample1X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(120))
//                .splineToLinearHeading(new Pose2d(46, -18, Math.toRadians(givingSampleAngle)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(sample2X, startSampleY), Math.toRadians(270), giveSampleVelConstraint)
                .splineToLinearHeading(new Pose2d((!do3rdSample ? 4 : 0) + giveSample2X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(!do3rdSample ? 270 : 120));
        if (do3rdSample)
            builder = builder
                    .splineToConstantHeading(new Vector2d(sample3X, startSampleY), Math.toRadians(270), giveSampleVelConstraint)
                    .splineToLinearHeading(new Pose2d(giveSample3X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270));
        return builder;
    }

    private static TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToY(scoreSpecimenY);
        return builder;
    }


}