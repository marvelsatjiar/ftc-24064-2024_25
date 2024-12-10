package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static boolean
                is5plus0 = false,
                usePartnerSpec = true;


    public static double
            scoreSpecimenY = -31,
            sample1X = 48,
            sample2X = 58,
            sample3X = 63,
            startSampleY = -14,
            bumpSpecimen = -64,
            yintakeSpecimen = -60,
            giveSample1X = sample1X - 4,
            giveSample2X = sample2X - 4,
            giveSample3X = sample3X,
            giveSampleY = -50,
            wallPickupX = 35,
            firstWallPickupX = 58,
            givingSampleAngle = 270;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1200);
        boolean isSpecimenSide = true;

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.75)
                .setDimensions(14,  16.5)
                .build();

        Pose2d startPose;
        startPose = new Pose2d(7.375,-62,Math.toRadians(270));


        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startPose);
        builder = scoreFirstSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);

        drive.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }

    public static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offset) {
        return builder
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(5 + offset, scoreSpecimenY, Math.toRadians(270)), Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(wallPickupX,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(bumpSpecimen);

    }

    private static TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .splineToSplineHeading(new Pose2d(firstWallPickupX, yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(firstWallPickupX, bumpSpecimen, Math.toRadians(270)), Math.toRadians(270));

        builder = scoreSpecimen(builder, 0);
        builder = scoreSpecimen(builder, -2);
        builder = scoreSpecimen(builder, -4);
        if (is5plus0)
            builder = scoreSpecimen(builder, -6);

        return builder;
    }
    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        boolean do3rdSample = is5plus0 || !usePartnerSpec;
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(33,-35),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(sample1X, startSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(giveSample1X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(120))
//                .splineToLinearHeading(new Pose2d(46, -18, Math.toRadians(givingSampleAngle)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(sample2X, startSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d((!do3rdSample ? 4 : 0) + giveSample2X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(!do3rdSample ? 270 : 120));
        if (do3rdSample)
            builder = builder
                    .splineToSplineHeading(new Pose2d(sample3X, startSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270))
                    .splineToLinearHeading(new Pose2d(giveSample3X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270));
        return builder;
    }

    private static TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToY(scoreSpecimenY);
        return builder;
    }


}