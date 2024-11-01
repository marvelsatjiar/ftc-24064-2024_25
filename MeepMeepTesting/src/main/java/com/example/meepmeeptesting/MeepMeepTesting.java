package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        boolean isSpecimenSide = false;

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(new Pose2d(-10,-63,Math.toRadians(270)));
        if (!isSpecimenSide) {
            builder = scoreSpecimen(builder);
            builder = scoreSample1(builder);
            builder = scoreSample2(builder);
            builder = scoreSample3(builder);
        } else {


        }

        drive.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }

    private static TrajectoryActionBuilder scoreSample2(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToXSplineHeading(-53,Math.toRadians(90))
                .setTangent(90)
                .lineToY(-43)
                .waitSeconds(1)
                .lineToYSplineHeading(-50,Math.toRadians(45))
                .waitSeconds(1)




                ;
        return builder;

    }

    private static TrajectoryActionBuilder scoreSample3(TrajectoryActionBuilder builder) {
        builder = builder
//                .lineToYSplineHeading(-40,Math.toRadians(115))
                .lineToXSplineHeading(-55,Math.toRadians(115))
//                .waitSeconds(1)
                .setTangent(90)
                .lineToY(-43)
                .waitSeconds(1)
                .lineToYSplineHeading(-50,Math.toRadians(55))




                ;



        return builder;
    }

    private static TrajectoryActionBuilder scoreSample1(TrajectoryActionBuilder builder) {
        builder = builder
//                .setTangent(-90)
                .lineToY(-40)
                .splineTo(new Vector2d(-48, -48), Math.toRadians(-270))
//                .setTangent(45)
                .waitSeconds(1)
                .setTangent(45)
                .splineToLinearHeading(new Pose2d(-55,-53,45),-270)
                .waitSeconds(1)
//                .setTangent(45)
//                .splineTo(new Vector2d(-53, -53), Math.toRadians(0))
//                .setTangent(-270)
//                .splineToL(new Vector2d(-55, -55), Math.toRadians(45))




        ;
        return builder;
    }

    private static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToY(-35)
                .waitSeconds(1);

        return builder;
    }


}