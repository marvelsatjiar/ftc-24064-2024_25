package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;

@Autonomous(name = "Specimen Side")
@Config
public class SpecimenAuto extends AbstractAuto {
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
            thirdSampleAngle3 = 0,
            extendoAngleSample1 = 65,
            retractExtendoWaitToWallPickup = 2,
            clampAfterSpecimenWait = 0.6,
            ySubmersibleSpecimen = -32,
            bumpSpecimen = -64,
            yintakeSpecimen = -60,
            giveSample1X = 41,
            giveSample2X = 46,
            giveSample3X = 52,
            giveSampleY = -57,
            wallPickupX = 35,
            bumpWall = -63,
            givingSampleAngle = -10;

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(startingPositionX, startingPositionY, Math.toRadians(-270));
    }

    @Override
    protected void onInit() {
        robot.arm.setArmAngle(Arm.ArmAngle.CHAMBER_FRONT_SETUP);
        robot.arm.setWristAngle(Arm.WristAngle.CHAMBER_FRONT);
        robot.claw.setAngle(Claw.ClawAngles.CLAMPED);

        robot.arm.run(false);
        robot.claw.run();
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

        builder = scoreFirstSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);

        return builder.build();
    }

    public static TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offset) {
        return builder
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(4 + offset,ySubmersibleSpecimen,Math.toRadians(270)),Math.toRadians(90))
                .stopAndAdd(RobotActions.scoreSpecimenFromFrontWallPickup())
                .afterTime(0.3, RobotActions.setupFrontWallPickup())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(wallPickupX,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(bumpSpecimen);
    }

    private static TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .splineToSplineHeading(new Pose2d(wallPickupX,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(wallPickupX, bumpWall, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0, RobotActions.takeSpecimenFromFrontWallPickup());

        builder = scoreSpecimen(builder, 0)
                .afterTime(0, RobotActions.takeSpecimenFromFrontWallPickup())
                .waitSeconds(clampAfterSpecimenWait);
        builder = scoreSpecimen(builder, -2)
                .afterTime(0, RobotActions.takeSpecimenFromFrontWallPickup())
                .waitSeconds(clampAfterSpecimenWait);
        builder = scoreSpecimen(builder, -4);
        return builder;
    }

    private static TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(15,-40),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(xSample1,ySample1, Math.toRadians(firstSampleAngle)), Math.toRadians(45))
                .afterTime(0,new SequentialAction(
                        RobotActions.setExtendo(extendoAngleSample1,0.1),
                        RobotActions.setV4B(Intake.V4BAngle.HOVERING,0)
                ))
                .splineToLinearHeading(new Pose2d(giveSample1X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(90))
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP,0))
                .splineToSplineHeading(new Pose2d(xSample2,ySample2, Math.toRadians(firstSampleAngle)), Math.toRadians(90))
                .afterTime(0,RobotActions.setV4B(Intake.V4BAngle.HOVERING,0))
                .splineToLinearHeading(new Pose2d(giveSample2X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(90))
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP,0))
                .splineToSplineHeading(new Pose2d(xSample3,ySample3, Math.toRadians(thirdSampleAngle3)), Math.toRadians(90))
                .afterTime(0,RobotActions.setV4B(Intake.V4BAngle.HOVERING,0))
                .splineToLinearHeading(new Pose2d(giveSample3X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270))
                .afterTime(0, new SequentialAction(
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0.1),
                        RobotActions.setExtendo(Extendo.Extension.RETRACTED, retractExtendoWaitToWallPickup),
                        RobotActions.setupFrontWallPickup()
                ));

        ;
        return builder;
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0.0, new ParallelAction(
                        RobotActions.setupChamberFromFront(),
                        RobotActions.setClaw(Claw.ClawAngles.CLAMPED, 0.0)
                ))
                .splineToConstantHeading(new Vector2d(scoreSpecimenX, scoreSpecimenY), Math.toRadians(90))
                .stopAndAdd(RobotActions.scoreChamberFromFrontAndRetract());
        return builder;
    }
}
