package org.firstinspires.ftc.teamcode.robot.into_the_deep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AbstractAuto.BACKWARD;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AbstractAuto.FORWARD;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Memory;
import org.firstinspires.ftc.teamcode.robot.into_the_deep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.into_the_deep.subsystem.Robot;

@TeleOp(group = "24064 main")
public final class MainTeleOp extends LinearOpMode {
    // Gamepads and the 'robot' class is imported to save lines and to import controls
    public static GamepadEx gamepadEx1;
    static Robot robot;
    public static GamepadEx gamepadEx2;
    public static MultipleTelemetry mTelemetry;

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

        Pose2d endPose = Memory.AUTO_END_POSE;
        if (endPose != null) {
            robot.drivetrain.setCurrentHeading(endPose.heading.toDouble() - (Memory.IS_RED ? FORWARD : BACKWARD));
        }

        waitForStart();

        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            robot.printTelemetry();
            robot.drivetrain.updatePoseEstimate();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            mTelemetry.update();

            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode
            double x = gamepadEx1.getRightX();
            if (gamepadEx1.isDown(LEFT_BUMPER)) {
                double y = gamepadEx1.getRightY();
                if (hypot(x, y) >= 0.8) robot.drivetrain.setCurrentHeading(atan2(y, x));
                x = 0;
            }

            double slowMult = gamepadEx1.isDown(RIGHT_BUMPER) ? 0.2 : 1;
            robot.drivetrain.setFieldCentricPowers(

                    new PoseVelocity2d(
                            new Vector2d(
                                    gamepadEx1.getLeftY() * slowMult,
                                    -gamepadEx1.getLeftX() * slowMult
                            ),
                            -x * slowMult
                    )
            );

            double stick = gamepadEx2.getRightY();
            if (stick != 0) robot.extendo.setWithStick(stick);

            if (gamepadEx2.wasJustPressed(DPAD_UP)) robot.intake.setTargetPoint(1);
            if (gamepadEx2.wasJustPressed(DPAD_DOWN)) robot.intake.setTargetPoint(2);
            if (gamepadEx2.wasJustPressed(DPAD_RIGHT)) robot.intake.setTargetPoint(3);

            double right_trig_pow = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double left_trig_pow = gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (right_trig_pow > 0) robot.intake.setServoPower(right_trig_pow);
            if (left_trig_pow > 0) robot.intake.setServoPower(-left_trig_pow);

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) robot.claw.toggleClaw();

        }
    }
}