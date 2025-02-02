package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx.SampleColor.BLUE;
import static org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx.SampleColor.RED;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement.AutoAligner;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;

@TeleOp(group = "24064 Main")
public final class MainTeleOp extends LinearOpMode {
    // Gamepads and the 'robot' class is imported to save lines and to import controls
    public static GamepadEx gamepadEx1, gamepadEx2;

    // Quick method that is used for better handling the controller
    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    @Override
    public void runOpMode() {
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap);

        Pose2d endPose = Common.AUTO_END_POSE;
        if (endPose != null) {
            robot.drivetrain.setCurrentHeading(endPose.heading.toDouble() - Common.FORWARD);
        }

        waitForStart();

        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();


            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode
            double x = gamepadEx1.getRightX();
            if (gamepadEx1.isDown(RIGHT_STICK_BUTTON)) {
                double y = gamepadEx1.getRightY();
                if (hypot(x, y) >= 0.8) robot.drivetrain.setCurrentHeading(atan2(y, x));
                x = 0;
            }

            double slowMult = gamepadEx1.isDown(LEFT_BUMPER) ? 0.3 : 1;

            double slowTurningMult = gamepadEx1.isDown(LEFT_BUMPER) ? 0.3 : 1;

            if (robot.extendo.getTargetExtension() != Extendo.Extension.RETRACTED) {
                slowMult = 0.3;
                slowTurningMult = 0.3;
            }

            PoseVelocity2d autoWallPickupPowers = null;
            if (gamepadEx1.isDown(X)) {
                autoWallPickupPowers = robot.autoWallPickUp.run(gamepadEx1.getLeftY() * slowMult, gamepadEx1.getLeftX() * slowMult);

                if (autoWallPickupPowers != null) {
                    robot.drivetrain.setDrivePowers(autoWallPickupPowers);
                }
            }

            if (autoWallPickupPowers == null) {
                robot.drivetrain.setFieldCentricPowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        gamepadEx1.getLeftY() * slowMult,
                                        -gamepadEx1.getLeftX() * slowMult
                                ),
                                -gamepadEx1.getRightX() * slowTurningMult
                        )
                );
            }

            if (keyPressed(1, B)) robot.drivetrain.setCurrentHeading(Math.PI);

            if (gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5) {
                robot.lift.runManual(gamepadEx2.getLeftY());
                robot.lift.reset();
            } else robot.lift.runManual((0));

            switch (robot.getCurrentState()) {
                // MISC ============================================================================
                case NEUTRAL:
                    doExtendoControls();
                    doIntakeControls();

                    if (keyPressed(2, Y)) robot.actionScheduler.addAction(RobotActions.setupFrontWallPickup());
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.transferToClaw());
                    if (keyPressed(2, A)) robot.actionScheduler.addAction(RobotActions.retractTransferAndSetupBasket());
                    if (keyPressed(2, LEFT_BUMPER)) robot.actionScheduler.addAction(RobotActions.setupLevelTwoHang());
                    break;
                case EXTENDO_OUT:
                    doExtendoControls();
                    doIntakeControls();

                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.transferToClaw());
                    if (keyPressed(2, Y)) robot.actionScheduler.addAction(RobotActions.retractExtendo());
                    if (keyPressed(2, A)) robot.actionScheduler.addAction(RobotActions.retractTransferAndSetupBasket());
                    break;
                case TRANSFERRED:
                    if (keyPressed(2, A)) robot.actionScheduler.addAction(RobotActions.setupScoreBasket(true));
                    if (keyPressed(2, Y)) robot.actionScheduler.addAction(RobotActions.setupChamberFromBack());
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.setupDropSample());
                    break;
                // BASKET ==========================================================================
                case SETUP_SCORE_BASKET:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.scoreBasket());
                    break;
                case SCORED_SAMPLE_HIGH_BASKET:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(new SequentialAction(
                            RobotActions.setArm(Arm.ArmAngle.NEUTRAL, 0.5),
                            RobotActions.retractToNeutral(0.2)));
                    break;
                // CHAMBER =========================================================================
                case SETUP_CHAMBER_FROM_FRONT:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.scoreChamberFromFrontAndRetract());
                    break;
                case SETUP_CHAMBER_FROM_BACK:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.retractToNeutral(0.5));
                    break;
                // WALL PICKUP =====================================================================
                case SETUP_FRONT_SPECIMEN_FROM_WALL:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.scoreSpecimenFromFrontWallPickup());
                    break;
                case FRONT_WALL_PICKUP:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.takeSpecimenFromFrontWallPickup(true));
                    break;
                // HANG ============================================================================
                case SETUP_LEVEL_TWO_HANG:
                    if (keyPressed(2, LEFT_BUMPER)) robot.actionScheduler.addAction(RobotActions.climbLevelTwoHang());
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.retractToNeutral(0.2));
                    break;
                case CLIMB_LEVEL_TWO_HANG:
                    if (keyPressed(2, LEFT_BUMPER)) robot.actionScheduler.addAction(RobotActions.setupLevelThreeHang());
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.retractToNeutral(0.2));
                    break;
                case SETUP_LEVEL_THREE_HANG:
                    if (keyPressed(2, LEFT_BUMPER)) robot.actionScheduler.addAction(RobotActions.climbLevelThreeHang());
                    break;
                // DROP SAMPLE =====================================================================
                case SETUP_DROP_SAMPLE:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.dropSample());
                    break;
            }

            robot.drivetrain.updatePoseEstimate();
            robot.run();
//            robot.printTelemetry();
        }
    }

    public void doExtendoControls() {
//        boolean isExtendoRetracted = robot.extendo.getTargetAngle() != Extendo.LINKAGE_MIN_ANGLE;
//        boolean isV4BDown = robot.intake.getTargetV4BAngle() == Intake.V4BAngle.DOWN;
//        boolean isAllianceSpecificSample = robot.intake.getCurrentSample() == RED && IS_RED || robot.intake.getCurrentSample() == BLUE && !IS_RED;
//        boolean isSampleYellow = robot.intake.getCurrentSample() == YELLOW;
//
//        if (isExtendoRetracted && isV4BDown) {
//            if (isSampleYellow) {
//                robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP, true);
//                robot.actionScheduler.addAction(RobotActions.transferToClaw());
//            }
//        }
//
//        if (isExtendoRetracted && isV4BDown) {
//            if (isAllianceSpecificSample) {
//                robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP, true);
//                robot.actionScheduler.addAction(RobotActions.retractForTransfer());
//            }
//        }

        if (keyPressed(2, DPAD_UP)) robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.EXTENDED));
        if (keyPressed(2, DPAD_LEFT)) robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH));
        if (keyPressed(2, DPAD_DOWN)) robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.ONE_HALF));
        if (keyPressed(2, DPAD_RIGHT)) robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.THREE_FOURTHS));
        if (gamepadEx2.getRightY() != 0) robot.extendo.setTargetAngleWithStick(-gamepadEx2.getRightY());
    }

    public void doIntakeControls() {
//        boolean isIntakeAlreadyPowered = robot.intake.getRollerPower() != -1;
//        boolean isV4BDown = robot.intake.getTargetV4BAngle() == Intake.V4BAngle.DOWN;
//        boolean isOppositeAllianceSample = robot.intake.getCurrentSample() == BLUE && IS_RED || robot.intake.getCurrentSample() == RED && !IS_RED;

//        if (isIntakeAlreadyPowered && isV4BDown) {
//            if (isOppositeAllianceSample) {
//                robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP, true);
//                robot.actionScheduler.addAction(RobotActions.setRollers(-1, 0.4));
//            }
//        }

        if (keyPressed(1, A)) robot.sweeper.toggleSweeper();

        if (!gamepadEx1.isDown(RIGHT_BUMPER) && !gamepadEx2.isDown(RIGHT_BUMPER)) {
            double trigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            if (trigger != 0) {
                robot.intake.setTargetV4BAngle(Intake.V4BAngle.DOWN);
                robot.intake.setRollerPower(trigger);
            } else {
                robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP);
                robot.intake.setRollerPower(0);
            }
        } else {
            robot.intake.setTargetV4BAngle(Intake.V4BAngle.CLEARING);
            robot.intake.setRollerPower(-1);
        }
    }

}