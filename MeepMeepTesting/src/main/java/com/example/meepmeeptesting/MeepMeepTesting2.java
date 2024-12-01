package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
    public static double
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -29,
            scoreSpecimenX = 4,
            firstSampleAngle = 35,
            xSample1 = 38,
            ySample1 = -36,
            xSample2 = 46,
            ySample2 = -35,
            xSample3 = 46,
            ySample3 = -40.5,
            bumpSample = -33.5,
            intakeSampleAngle3 = 29.25,
            extendoAngleSample3 = 85,
            bumpSample3 = -35.5,
            xSubmersibleSpecimen = 6,
            ySubmersibleSpecimen = -30,
            xintakeSpecimen = 16,
            yintakeSpecimen = -46;


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.75)
                .setDimensions(14,  16.5)
                .build();


        Pose2d startPose = new Pose2d(7.375,-62,Math.toRadians(-270));


        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startPose);
        builder = scoreSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);
    }

    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(26,-38),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(xSample1,ySample1,Math.toRadians(firstSampleAngle)),Math.toRadians(35))
                .lineToY(bumpSample)
                .splineToLinearHeading(new Pose2d((xSample2 + xSample1) / 2, ySample2, Math.toRadians(300)), Math.toRadians(300))
                // Sample 2 intake
                .splineToLinearHeading(new Pose2d(xSample2,ySample2,Math.toRadians(25)),Math.toRadians(35))
                .lineToY(bumpSample)
                .splineToLinearHeading(new Pose2d((xSample2 + xSample1) / 2, ySample2, Math.toRadians(300)), Math.toRadians(300))

                .splineToLinearHeading(new Pose2d(xSample3, ySample3, Math.toRadians(intakeSampleAngle3)), Math.toRadians(10))

                .lineToY(bumpSample3)
                // Intake sample 3
                .splineToLinearHeading(new Pose2d((xSample2 + xSample1) / 2, ySample2, Math.toRadians(300)), Math.toRadians(300))
        ;
        // Given Sample 3;
        ;
        return builder;
    }

    private static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .splineToConstantHeading(new Vector2d(4,-29), Math.toRadians(90));

        return builder;
    }

    private static TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {
        return null;
    }


}
