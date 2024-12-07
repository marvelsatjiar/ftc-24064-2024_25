package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -29,
            scoreSpecimenX = 4,
            firstSampleAngle = 30,
            xSample1 = 30,
            ySample1 = -32,
            xSample2 = 46,
            ySample2 = -42,
            xSample3 = 54,
            ySample3 = -31,
            bumpSample = -33.5,
            intakeSampleAngle3 = 29.25,
            extendoAngleSample1 = 85,
            bumpSample3 = 48.5,
            xSubmersibleSpecimen = 7,
            ySubmersibleSpecimen = -36,
            xintakeSpecimen = 32.5,
            giveSample1X = 27,
            giveSample2X = 44,
            giveSample3X = 54,
            giveSampleY = -46,
            yintakeSpecimen = -60,
            bumpWall = -63;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        boolean isSpecimenSide = true;

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.75)
                .setDimensions(14,  16.5)
                .build();

        Pose2d startPose;
        if (isSpecimenSide)
            startPose = new Pose2d(7.375,-62,Math.toRadians(-270));
        else
            startPose = new Pose2d(-7.375,-62,Math.toRadians(-270));




        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startPose);
        if (!isSpecimenSide) {
            builder = scoreSpecimen(builder);
            builder = scoreSamples(builder);
            builder = samplePark(builder);
        } else {
            builder = scoreSpecimen(builder);
            builder = giveSamples(builder);
            builder = scoreAllSpecimens2(builder);
//            builder = scoreAllSpecimens(builder);
//            builder = specimenPark(builder);





        }

        drive.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }

    private static TrajectoryActionBuilder scoreAllSpecimens2(TrajectoryActionBuilder builder) {
        builder = builder
                // Intaking 1st Specimen
                .splineToLinearHeading(new Pose2d(37,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(bumpWall)
//                .lineToYLinearHeading(yintakeSpecimen,Math.toRadians(270))
                .setTangent(Math.toRadians(135))
                //Going to Sub
                .splineToLinearHeading(new Pose2d(4,ySubmersibleSpecimen,Math.toRadians(270)),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(37,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(90))
//                .setTangent(Math.toRadians(135))
//                .splineToLinearHeading(new Pose2d(4,ySubmersibleSpecimen,Math.toRadians(270)),Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(37,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))
//                .setTangent(Math.toRadians(135))
//                .splineToLinearHeading(new Pose2d(4,ySubmersibleSpecimen,Math.toRadians(270)),Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(37,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))

        ;
        return builder;
    }

    private static TrajectoryActionBuilder specimenPark(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToYLinearHeading(-48,Math.toRadians(-40))
        ;

        return builder;
    }

    private static TrajectoryActionBuilder samplePark(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25,-10,Math.toRadians(180)),Math.toRadians(0))
        ;
        return builder;
    }

    private static TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(17,-46, Math.toRadians(-45)),Math.toRadians(180))
                .lineToXConstantHeading(19)
                .splineToLinearHeading(new Pose2d(7,-34, Math.toRadians(-90)),Math.toRadians(180))
//                .setTangent(-45)
//                .lineToYLinearHeading(-45,Math.toRadians(-40))
//                .lineToYLinearHeading(-36,Math.toRadians(270))
//                .lineToYLinearHeading(-45,Math.toRadians(-40))
//                .lineToYLinearHeading(-36,Math.toRadians(270))
//                .lineToYLinearHeading(-45,Math.toRadians(-40))
//                .lineToYLinearHeading(-36,Math.toRadians(270))
        ;
        return builder;


    }
    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(26,-40),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xSample1,ySample1, Math.toRadians(firstSampleAngle)), Math.toRadians(35))
                .splineToLinearHeading(new Pose2d(giveSample1X,giveSampleY, Math.toRadians(0)), Math.toRadians(35))
                .splineToLinearHeading(new Pose2d(xSample2,ySample2, Math.toRadians(firstSampleAngle)), Math.toRadians(35))
                .splineToLinearHeading(new Pose2d(giveSample2X,giveSampleY, Math.toRadians(0)), Math.toRadians(35))
                .splineToLinearHeading(new Pose2d(xSample3,ySample3, Math.toRadians(firstSampleAngle)), Math.toRadians(35))
                .splineToLinearHeading(new Pose2d(giveSample3X,giveSampleY, Math.toRadians(0)), Math.toRadians(35))





        ;
        return builder;
    }

    private static TrajectoryActionBuilder scoreSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(-35,-38),Math.toRadians(180))`
                .splineToSplineHeading(new Pose2d(-43.5,-33.2,Math.toRadians(-235)),Math.toRadians(180))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-53.5,-53.5,Math.toRadians(45)),Math.toRadians(-135))
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(-58,-34,Math.toRadians(90)),Math.toRadians(110))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)),Math.toRadians(290))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-49,-44,Math.toRadians(145)),Math.toRadians(145))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-51, -42), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)),Math.toRadians(255))
        ;
        return builder;
    }

    private static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .splineToConstantHeading(new Vector2d(scoreSpecimenX,scoreSpecimenY), Math.toRadians(90))
                ;

        return builder;
    }


}