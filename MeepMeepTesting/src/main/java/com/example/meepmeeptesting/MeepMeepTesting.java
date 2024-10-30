package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(new Pose2d(17,-70,Math.toRadians(90)));
        if (!isSpecimenSide) {
            builder = scoreSpecimen(builder);
            builder = intake1(builder);
            builder = intake2(builder);
            builder = intake3(builder);
        } else {


        }

        drive.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }

    private static TrajectoryActionBuilder intake2(TrajectoryActionBuilder builder) {
        return builder;
    }

    private static TrajectoryActionBuilder intake3(TrajectoryActionBuilder builder) {
        return builder;
    }

    private static TrajectoryActionBuilder intake1(TrajectoryActionBuilder builder) {
        return builder;
    }

    private static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToY(17);

        return builder;
    }


}