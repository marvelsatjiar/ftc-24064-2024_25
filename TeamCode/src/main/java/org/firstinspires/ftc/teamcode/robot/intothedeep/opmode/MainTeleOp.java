package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AbstractAuto.BACKWARD;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AbstractAuto.FORWARD;
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

import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Memory;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.ITDRobot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;

@TeleOp(group = "24064 main")
public final class MainTeleOp extends LinearOpMode {
    // Gamepads and the 'robot' class is imported to save lines and to import controls
    public static GamepadEx gamepadEx1;
    static ITDRobot itdRobot;
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

        itdRobot = new ITDRobot(hardwareMap);

        Pose2d endPose = Memory.AUTO_END_POSE;
        if (endPose != null) {
            itdRobot.drivetrain.setCurrentHeading(endPose.heading.toDouble() - (Memory.IS_RED ? FORWARD : BACKWARD));
        }

        waitForStart();

        while (opModeIsActive()) {
            // Read sensors + gamepads:
            itdRobot.readSensors();
            itdRobot.printTelemetry();
            itdRobot.drivetrain.updatePoseEstimate();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            mTelemetry.update();

            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode
            double x = gamepadEx1.getRightX();
            if (gamepadEx1.isDown(LEFT_BUMPER)) {
                double y = gamepadEx1.getRightY();
                if (hypot(x, y) >= 0.8) itdRobot.drivetrain.setCurrentHeading(atan2(y, x));
                x = 0;
            }

            double slowMult = gamepadEx1.isDown(RIGHT_BUMPER) ? 0.2 : 1;
            itdRobot.drivetrain.setFieldCentricPowers(

                    new PoseVelocity2d(
                            new Vector2d(
                                    gamepadEx1.getLeftY() * slowMult,
                                    -gamepadEx1.getLeftX() * slowMult
                            ),
                            -x * slowMult
                    )
            );

            double stick = gamepadEx2.getRightY();
            if (stick != 0) itdRobot.extendo.setWithStick(stick);

            if (gamepadEx2.wasJustPressed(DPAD_UP)) itdRobot.intake.setTarget(Intake.V4BAngles.UP);
            if (gamepadEx2.wasJustPressed(DPAD_DOWN)) itdRobot.intake.setTarget(Intake.V4BAngles.DOWN);
            if (gamepadEx2.wasJustPressed(DPAD_RIGHT)) itdRobot.intake.setTarget(Intake.V4BAngles.CLEARING);

            double trigger1 = gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER);
            double trigger2 = gamepadEx2.getTrigger(RIGHT_TRIGGER) - gamepadEx2.getTrigger(LEFT_TRIGGER);
            double intake = trigger1 != 0 ? trigger1 : trigger2;

            itdRobot.intake.setServoPower(intake);



            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) itdRobot.claw.toggleClaw();
        }
    }
}