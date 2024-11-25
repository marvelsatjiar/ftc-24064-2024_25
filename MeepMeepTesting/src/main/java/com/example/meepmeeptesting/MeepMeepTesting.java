package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        boolean isSpecimenSide = false;

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.75)
                .build();

        Pose2d startPose;
        if (isSpecimenSide)
            startPose = new Pose2d(10,-63,Math.toRadians(-270));
        else
            startPose = new Pose2d(-8,-63,Math.toRadians(-270));




        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startPose);
        if (!isSpecimenSide) {
            builder = scoreSpecimen(builder);
            builder = scoreSamples(builder);
            builder = samplePark(builder);
        } else {
            builder = scoreSpecimen(builder);
            builder = giveSamples(builder);
            builder = scoreAllSpecimens(builder);
            builder = specimenPark(builder);





        }

        drive.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
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
                .splineToConstantHeading(new Vector2d(26,-38),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(7,-36),Math.toRadians(180))
                .setTangent(-45)
                .lineToYLinearHeading(-45,Math.toRadians(-40))
                .lineToYLinearHeading(-36,Math.toRadians(270))
                .lineToYLinearHeading(-45,Math.toRadians(-40))
                .lineToYLinearHeading(-36,Math.toRadians(270))
                .lineToYLinearHeading(-45,Math.toRadians(-40))
                .lineToYLinearHeading(-36,Math.toRadians(270))
        ;
        return builder;


    }
    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(-20))
                .splineToConstantHeading(new Vector2d(26,-38),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(35,-36,Math.toRadians(35)),Math.toRadians(35))
        //extend intake ~5in
                .turn(Math.toRadians(-70))
                //give sample 2 below
                .splineToSplineHeading(new Pose2d(47,-36,Math.toRadians(35)),Math.toRadians(35))
                .turn(Math.toRadians(-80))
        //give sample 3 below
                .turn(Math.toRadians(65))
                .turn(Math.toRadians(-110))
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
                .splineToLinearHeading(new Pose2d(-54,-46,Math.toRadians(127)),Math.toRadians(127))
                .setTangent(Math.toRadians(255))
                .splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)),Math.toRadians(255))
        ;
        return builder;
    }

    private static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToY(-31)
                ;

        return builder;
    }


}