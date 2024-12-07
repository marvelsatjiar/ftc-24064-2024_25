package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
            robot.drivetrain.setCurrentHeading(endPose.heading.toDouble() - (Common.IS_RED ? Common.BACKWARD : Common.FORWARD));
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

//            if (robot.autoAligner.getTargetDistance() != AutoAligner.TargetDistance.INACTIVE) {
//                robot.drivetrain.setFieldCentricPowers(
//                                robot.autoAligner.run(gamepadEx1.getLeftX())
//                );
//            } else {
                robot.drivetrain.setFieldCentricPowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        gamepadEx1.getLeftY() * slowMult,
                                        -gamepadEx1.getLeftX() * slowMult
                                ),
                                -gamepadEx1.getRightX() * slowMult
                        )
                );
//            }

//            if (keyPressed(1, A)) robot.actionScheduler.addAction(RobotActions.alignRobotWithSensor(AutoAligner.TargetDistance.SUBMERSIBLE));
//            if (keyPressed(1, B)) robot.actionScheduler.addAction(RobotActions.alignRobotWithSensor(AutoAligner.TargetDistance.WALL_PICKUP));
//            if (keyPressed(1, X)) robot.actionScheduler.addAction(RobotActions.alignRobotWithSensor(AutoAligner.TargetDistance.CLIMB));


            switch (robot.getCurrentState()) {
                case TRANSFERRED:
                    if (keyPressed(2, A)) robot.actionScheduler.addAction(RobotActions.setupScoreBasket(true));
                    if (keyPressed(2, Y)) robot.actionScheduler.addAction(RobotActions.setupChamberFromBack());
                    break;
                case SETUP_SCORE_BASKET:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.scoreBasket());
                    break;
                case SCORED_SAMPLE_HIGH_BASKET:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.retractToNeutral(0.2));
                case SETUP_CHAMBER_FROM_FRONT:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.scoreChamberFromFrontAndRetract());
                    break;
                case SETUP_CHAMBER_FROM_BACK:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.retractToNeutral(0.5));
                    break;
                case NEUTRAL:
                    doExtendoControls();
                    doIntakeControls();

                    if (keyPressed(2, Y)) robot.actionScheduler.addAction(RobotActions.setupFrontWallPickup());
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.transferToClaw());
                    if (keyPressed(2, LEFT_BUMPER)) robot.actionScheduler.addAction(RobotActions.setupLevelTwoHang());
                    break;
                case EXTENDO_OUT:
                    doExtendoControls();
                    doIntakeControls();
//
//                    if (keyPressed(2, Y)) robot.actionScheduler.addAction(RobotActions.retractForTransfer());
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.transferToClaw());
                    break;
                case TO_BE_TRANSFERRED:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.transferToClaw());
                    break;
                case FRONT_WALL_PICKUP:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.setupSpecimenFromFrontWallPickup());
                    break;
                case SETUP_FRONT_SPECIMEN_FROM_WALL:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.scoreSpecimenFromFrontWallPickup());
                    break;
                case SETUP_LEVEL_TWO_HANG:
                    if (keyPressed(2, LEFT_BUMPER)) robot.actionScheduler.addAction(RobotActions.climbLevelTwoHang());
                    break;
                case CLIMB_LEVEL_TWO_HANG:
                    if (keyPressed(2, LEFT_BUMPER)) robot.actionScheduler.addAction(RobotActions.setupLevelThreeHang());
                    break;
                case SETUP_LEVEL_THREE_HANG:
                    if (keyPressed(2, LEFT_BUMPER)) robot.actionScheduler.addAction(RobotActions.climbLevelThreeHang());
                    break;
            }

            robot.drivetrain.updatePoseEstimate();
            robot.run();
            robot.printTelemetry();
        }
    }

    public void doExtendoControls() {
        if (keyPressed(2, DPAD_UP)) robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.EXTENDED));
        if (keyPressed(2, DPAD_LEFT)) robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH));
        if (keyPressed(2, DPAD_DOWN)) robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.ONE_HALF));
        if (keyPressed(2, DPAD_RIGHT)) robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.THREE_FOURTHS));
    }

    public void doIntakeControls() {
//        if (robot.intake.getRollerPower() != -1 && ((robot.intake.currentSample == Intake.SampleColor.BLUE && Common.IS_RED) || (robot.intake.currentSample == Intake.SampleColor.RED && !Common.IS_RED))) robot.actionScheduler.addAction(RobotActions.setRollers(-1, 1));

        if (!gamepadEx1.isDown(RIGHT_BUMPER)) {
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
        }
    }

}