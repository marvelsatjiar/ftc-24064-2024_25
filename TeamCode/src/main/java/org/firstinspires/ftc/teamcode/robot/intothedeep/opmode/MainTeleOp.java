package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Lift;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

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
            robot.drivetrain.setCurrentHeading(endPose.heading.toDouble() - (Common.IS_RED ? Common.FORWARD : Common.BACKWARD));
        }

        waitForStart();

        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();


            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode

            double slowMult = gamepadEx1.isDown(RIGHT_BUMPER) ? 0.2 : 1;
            robot.drivetrain.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    gamepadEx1.getLeftY() * slowMult,
                                    -gamepadEx1.getLeftX() * slowMult
                            ),
                            -gamepadEx1.getRightX() * slowMult
                    )
            );

            switch (robot.getCurrentState()) {
                case TRANSFERRED:
                    if (keyPressed(2, Y)) robot.actionScheduler.addAction(RobotActions.setupScoreBasket(true));
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.scoreBasketAndRetract(true));
                    break;
                case SETUP_SCORE_BASKET :
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.scoreBasketAndRetract(true));
                    break;
                case NEUTRAL:
                    if (keyPressed(2, DPAD_UP)) robot.actionScheduler.addAction(RobotActions.extendIntake());
                    break;
                case SETUP_INTAKE:
                    if (!gamepadEx1.isDown(LEFT_BUMPER)) {
                        double trigger = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
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

                    if (keyPressed(2, Y)) robot.actionScheduler.addAction(RobotActions.retractForTransfer());
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.transferToClaw());
                    break;
                case TO_BE_TRANSFERRED:
                    if (keyPressed(2, X)) robot.actionScheduler.addAction(RobotActions.transferToClaw());
                    break;
            }

            robot.run();
            robot.printTelemetry();
        }
    }
}