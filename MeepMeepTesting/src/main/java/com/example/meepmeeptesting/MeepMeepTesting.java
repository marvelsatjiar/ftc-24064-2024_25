package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -31,
            scoreSpecimenX = 4,
            firstSampleAngle = 30,
            xSample1 = 28,
            ySample1 = -32,
            xSample2 = 37,
            ySample2 = -31,
            xSample3 = 47,
            ySample3 = -26,
            bumpSample = -33.5,
            thirdSampleAngle3 = 0,
            extendoAngleSample1 = 65,
            bumpSample3 = -35.5,
            retractExtendoWait = 0.3,
            retractExtendoWaitToWallPickup = 2,
            clampAfterSpecimenWait = 0.6,
            xSubmersibleSpecimen = 5,
            ySubmersibleSpecimen = -32,
            xintakeSpecimen = 32.5,
            bumpSpecimen = -64,
            yintakeSpecimen = -60,
            yintakeSpecimenWall = -62.5,
            transferSpecimenToClawWait = 0.5,
            rollerIntakeSeconds = 1,
            setupChamberFromBackWait = 1,
            bumpDelay = 0,
            wristSpecimenWait = 0.3,
            giveSample1X = 41,
            giveSample2X = 46,
            giveSample3X = 52,
            giveSampleY = -57,
            wallPickupX = 35,
            bumpWall = -63,
            unclampSpecimenWait = 0.1,
            givingSampleAngle = -10;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        boolean isSpecimenSide = true;

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.75)
                .setDimensions(14,  16.5)
                .build();

        Pose2d startPose;
        startPose = new Pose2d(7.375,-62,Math.toRadians(-270));


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
                .splineToLinearHeading(new Pose2d(4 + offset,ySubmersibleSpecimen,Math.toRadians(270)),Math.toRadians(90))
//                .stopAndAdd(RobotActions.scoreSpecimenFromFrontWallPickup())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(wallPickupX,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(bumpSpecimen);

    }

    private static TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .splineToSplineHeading(new Pose2d(wallPickupX,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(wallPickupX, bumpWall, Math.toRadians(270)), Math.toRadians(270));

        builder = scoreSpecimen(builder, 0);
        builder = scoreSpecimen(builder, -2);
        builder = scoreSpecimen(builder, -4);
        return builder;
    }

    private static TrajectoryActionBuilder specimenPark(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToYLinearHeading(-48,Math.toRadians(-40))
                // Extend fully
        ;
        return builder;
    }
    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(15,-40),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(xSample1,ySample1, Math.toRadians(firstSampleAngle)), Math.toRadians(45))
//                .afterTime(0,new SequentialAction(
//                        RobotActions.setExtendo(extendoAngleSample1,0.1),
//                        RobotActions.setV4B(Intake.V4BAngle.HOVERING,0)
//                ))
                .splineToLinearHeading(new Pose2d(giveSample1X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(90))
//                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP,0))
                .splineToSplineHeading(new Pose2d(xSample2,ySample2, Math.toRadians(firstSampleAngle)), Math.toRadians(90))
//                .afterTime(0,RobotActions.setV4B(Intake.V4BAngle.HOVERING,0))
                .splineToLinearHeading(new Pose2d(giveSample2X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(90))
//                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP,0))
                .splineToSplineHeading(new Pose2d(xSample3,ySample3, Math.toRadians(thirdSampleAngle3)), Math.toRadians(90))
//                .afterTime(0,RobotActions.setV4B(Intake.V4BAngle.HOVERING,0))
                .splineToLinearHeading(new Pose2d(giveSample3X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270))
//                .afterTime(0, new SequentialAction(
//                        RobotActions.setV4B(Intake.V4BAngle.UP,0.1),
//                        RobotActions.setExtendo(Extendo.Extension.RETRACTED, retractExtendoWaitToWallPickup),
//                        RobotActions.setupFrontWallPickup()
//                ));

        ;
        return builder;
    }

    private static TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .splineToConstantHeading(new Vector2d(scoreSpecimenX,scoreSpecimenY), Math.toRadians(90))
                ;

        return builder;
    }


}