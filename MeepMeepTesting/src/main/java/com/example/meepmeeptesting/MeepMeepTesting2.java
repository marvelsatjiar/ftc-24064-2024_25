package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {


    public static double
            xBasket1 = -58,
            yBasket1 = -51,
            xBasket2 = -62,
            yBasket2 = -51,
            xIntakeSample3 = -51,
            yIntakeSample3 = -46,
            subX = -25,
            subY = -12,
            xBasketSub = -53,
            yBasketSub = -53,

            sample5thOffset = 1,
            sample6thOffset = 1,
            sample7thOffset = 1,
            sample8thOffset = 1,


            intake1stSampleAngle = 68,
            intake2ndSampleAngle = 82,
            intake3rdSampleAngle = 125,

            basketAngle = 45;




    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 11.75)
                .setDimensions(14, 16.5)
                .build();


        Pose2d startSamplePose;
        startSamplePose = new Pose2d(-39.0, -63.375, Math.toRadians(0));

        TrajectoryActionBuilder builder = drive.getDrive().actionBuilder(startSamplePose);
        builder = scoreFirstFourSamples(builder);
        builder = scoreAllSubSamples(builder);

        drive.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }

    private static TrajectoryActionBuilder scoreFirstFourSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .strafeToLinearHeading(new Vector2d(xBasket1, yBasket1), Math.toRadians(intake1stSampleAngle))
                .strafeToLinearHeading(new Vector2d(xBasket2, yBasket2), Math.toRadians(intake2ndSampleAngle))
                .strafeToLinearHeading(new Vector2d(xIntakeSample3, yIntakeSample3), Math.toRadians(intake3rdSampleAngle))
                .strafeToLinearHeading(new Vector2d(xBasketSub, yBasketSub), Math.toRadians(basketAngle));
        return builder;
    }

    private static TrajectoryActionBuilder scoreSubSamples(TrajectoryActionBuilder builder, double offsetY) {
        builder = builder
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(subX, subY, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xBasketSub, yBasketSub, Math.toRadians(basketAngle)), Math.toRadians(-135));
        return builder;
    }


    private static TrajectoryActionBuilder scoreAllSubSamples(TrajectoryActionBuilder builder) {
        builder = scoreSubSamples(builder, sample5thOffset);
        builder = scoreSubSamples(builder, sample6thOffset);
        builder = scoreSubSamples(builder, sample7thOffset);
        builder = scoreSubSamples(builder, sample8thOffset);
        return builder;

    }

}

