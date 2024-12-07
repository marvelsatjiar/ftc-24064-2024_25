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
            xSample3 = 49 ,
            ySample3 = -26,
            bumpSample = -33.5,
            thirdSampleAngle3 = 5,
            extendoAngleSample1 = 65,
            bumpSample3 = -35.5,
            retractExtendoWait = 0.3,
            retractExtendoWaitToWallPickup = 2,
            clampAfterSpecimenWait = 0.6,
            xSubmersibleSpecimen = 5,
            ySubmersibleSpecimen = -30,
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
            giveSample3X = 49,
            giveSampleY = -57,
            wallPickupX = 35,
            unclampSpecimenWait = 0.1;

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
        builder = scoreAllSpecimensWallPickUp(builder);
//        builder = park(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder scoreAllSpecimensWallPickUp(TrajectoryActionBuilder builder) {
        builder = builder
                // Intaking 1st Specimen
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(wallPickupX,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(bumpSpecimen)
                .waitSeconds(clampAfterSpecimenWait)
                .stopAndAdd(RobotActions.setupSpecimenFromFrontWallPickup())
                //Going to Sub
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(4,ySubmersibleSpecimen,Math.toRadians(270)),Math.toRadians(270))
                .stopAndAdd(RobotActions.scoreSpecimenFromFrontWallPickup())
                .splineToLinearHeading(new Pose2d(wallPickupX,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(90))
                .lineToY(bumpSpecimen)
                .waitSeconds(clampAfterSpecimenWait)
                .stopAndAdd(RobotActions.setupSpecimenFromFrontWallPickup())
        // Intaking 2nd Specimen
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(4,ySubmersibleSpecimen,Math.toRadians(270)),Math.toRadians(270))
                .stopAndAdd(RobotActions.scoreSpecimenFromFrontWallPickup())
                .splineToLinearHeading(new Pose2d(wallPickupX,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(90))
                .lineToY(bumpSpecimen)
                .waitSeconds(clampAfterSpecimenWait)
                .stopAndAdd(RobotActions.setupSpecimenFromFrontWallPickup())
                //Going to Sub+
                .splineToLinearHeading(new Pose2d(wallPickupX,yintakeSpecimen, Math.toRadians(270)), Math.toRadians(90))
                .stopAndAdd(RobotActions.scoreSpecimenFromFrontWallPickup());
        // Intaking 3rd Specimen

        return builder;
    }

    private TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToYLinearHeading(-48,Math.toRadians(-40));

        return builder;
    }

//    private TrajectoryActionBuilder scoreAllSpecimensExtendo(TrajectoryActionBuilder builder) {
//        builder = builder
//                // Turning to intake 1st Specimen
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(xintakeSpecimen,yintakeSpecimen,Math.toRadians(-45)), Math.toRadians(180))
//                .stopAndAdd( new SequentialAction(
//                        RobotActions.setV4B(Intake.V4BAngle.DOWN,0.1),
//                        RobotActions.setRollers(1, rollerIntakeSeconds)))
//                .lineToXConstantHeading(bumpSpecimen)
//                .afterTime(0.0, new SequentialAction(
//                        RobotActions.transferToClaw(),
//                        RobotActions.setupChamberFromBack()
//                ))
//                .waitSeconds(setupChamberFromBackWait)
//        // Moving to chamber
//                .splineToLinearHeading(new Pose2d(xSubmersibleSpecimen, ySubmersibleSpecimen, Math.toRadians(-90)),Math.toRadians(180))
//                // Scoring 1st Specimen
//                .stopAndAdd(scoreBackSpecimen())
////                // Moving to intake 2nd Specimen
//                .setTangent(-45)
//
////                .afterTime(unclampAfterTimeWait, new SequentialAction(
////                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, UnclampSpecimenWait),
////                        RobotActions.setClaw(Claw.ClawAngles.DEPOSIT_ANGLE, 0),
////                        RobotActions.retractToNeutral(0)
////                ))
//
//
//
//
//                .lineToYLinearHeading(-45,Math.toRadians(-45))
//                .afterTime(bumpDelay, new SequentialAction(
//                        RobotActions.setV4B(Intake.V4BAngle.DOWN,0.1),
//                        RobotActions.setRollers(1, rollerIntakeSeconds)))
//                .lineToX(bumpSpecimen)
//                .afterTime(0.0, new SequentialAction(
//                        RobotActions.transferToClaw(),
//                        RobotActions.setupChamberFromBack()
//                ))
//                // Scoring 2nd Specimen
//                .waitSeconds(setupChamberFromBackWait)
//                .splineToLinearHeading(new Pose2d(xSubmersibleSpecimen - 1, ySubmersibleSpecimen, Math.toRadians(-90)),Math.toRadians(180))
//                .stopAndAdd(scoreBackSpecimen())
//                .setTangent(-45)
//
////                .afterTime(unclampAfterTimeWait, new SequentialAction(
////                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, UnclampSpecimenWait),
////                        RobotActions.setClaw(Claw.ClawAngles.DEPOSIT_ANGLE, 0),
////                        RobotActions.retractToNeutral(0)
////                ))
//                .lineToYLinearHeading(-45,Math.toRadians(-45))
//
//                .afterTime(bumpDelay, new SequentialAction(
//                        RobotActions.setV4B(Intake.V4BAngle.DOWN,0.1),
//                        RobotActions.setRollers(1, rollerIntakeSeconds)))
//                .lineToX(bumpSpecimen)
//                .afterTime(0.0, new SequentialAction(
//                        RobotActions.transferToClaw(),
//                        RobotActions.setupChamberFromBack()
//                ))
//                // Scoring 2nd Specimen
//                .waitSeconds(setupChamberFromBackWait)
//                .splineToLinearHeading(new Pose2d(xSubmersibleSpecimen - 2, ySubmersibleSpecimen, Math.toRadians(-90)),Math.toRadians(180))
//                .stopAndAdd(scoreBackSpecimen())
//                .setTangent(-45)
//
//
////                .afterTime(unclampAfterTimeWait, new SequentialAction(
////                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, UnclampSpecimenWait),
////                        RobotActions.setClaw(Claw.ClawAngles.DEPOSIT_ANGLE, 0),
////                        RobotActions.retractToNeutral(0)
////                ))
////
////                .lineToYLinearHeading(-45,Math.toRadians(-45))
//
////                .stopAndAdd( new SequentialAction(
////                        RobotActions.setV4B(Intake.V4BAngle.DOWN,0.1),
////                        RobotActions.setRollers(1, rollerIntakeSeconds)))
//        ;
//
//        // Moving to intake 3rd Specimen
////                .afterTime(0.0, RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH))
////                .lineToYLinearHeading(-45,Math.toRadians(-40))
////                .stopAndAdd(RobotActions.setRollers(1, 0.5))
////                .afterTime(0.0, new SequentialAction(
////                        RobotActions.transferToClaw(),
////                        RobotActions.setupChamberFromBack()
////                ))
////                // Scoring 3rd Specimen
////                .lineToYLinearHeading(-36,Math.toRadians(270))
////                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract());
//        return builder;
//    }

//    private Action scoreBackSpecimen() {
//        return new ParallelAction(
//                new SequentialAction(
//                        RobotActions.setWrist(Arm.WristAngle.CHAMBER_BACK_AUTON, wristSpecimenWait),
//                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, unclampSpecimenWait),
//                        RobotActions.setClaw(Claw.ClawAngles.DEPOSIT, 0),
//                        RobotActions.retractToNeutral(0)
//                ),
//                new SequentialAction(
//                        new SleepAction(transferSpecimenToClawWait),
//                        RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0)
//                )
//        );
//    }

    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(26,-40),Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(xSample1,ySample1, Math.toRadians(firstSampleAngle)), Math.toRadians(35))

                .afterTime(0,new SequentialAction(
                        RobotActions.setExtendo(extendoAngleSample1,0.1),
                        RobotActions.setV4B(Intake.V4BAngle.HOVERING,0)
                    ))
                .splineToLinearHeading(new Pose2d(giveSample1X,giveSampleY, Math.toRadians(0)), Math.toRadians(35))
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP,0))
                .splineToLinearHeading(new Pose2d(xSample2,ySample2, Math.toRadians(firstSampleAngle)), Math.toRadians(35))
                .afterTime(0,RobotActions.setV4B(Intake.V4BAngle.HOVERING,0))
                .splineToLinearHeading(new Pose2d(giveSample2X,giveSampleY, Math.toRadians(0)), Math.toRadians(35))
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP,0))
                .splineToLinearHeading(new Pose2d(xSample3,ySample3, Math.toRadians(thirdSampleAngle3)), Math.toRadians(35))
                .afterTime(0,RobotActions.setV4B(Intake.V4BAngle.HOVERING,0))
                .splineToLinearHeading(new Pose2d(giveSample3X,giveSampleY, Math.toRadians(0)), Math.toRadians(35))
                .afterTime(0, new SequentialAction(
                        RobotActions.setV4B(Intake.V4BAngle.UP,0.1),
                        RobotActions.setExtendo(Extendo.Extension.RETRACTED, retractExtendoWaitToWallPickup),
                        RobotActions.setupFrontWallPickup()
                ));

        return builder;
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0.0, new ParallelAction(
                        RobotActions.setupChamberFromFront(),
                        RobotActions.setClaw(Claw.ClawAngles.CLAMPED, 0.0)
                ))
                .splineToConstantHeading(new Vector2d(scoreSpecimenX, scoreSpecimenY), Math.toRadians(90))
                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract());
        return builder;
    }
}
