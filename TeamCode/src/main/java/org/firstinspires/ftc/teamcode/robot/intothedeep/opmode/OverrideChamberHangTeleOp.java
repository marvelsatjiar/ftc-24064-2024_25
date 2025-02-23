package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Lift;

@TeleOp(group = "24064 Main")
public class OverrideChamberHangTeleOp extends LinearOpMode {
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    MecanumDrive drivetrain;
    Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        lift = new Lift(hardwareMap);

        Pose2d endPose = Common.AUTO_END_POSE;
        if (endPose != null) {
            drivetrain.setCurrentHeading(endPose.heading.toDouble() - Common.FORWARD);
        }

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            lift.setTargetTicks(Lift.Ticks.CHAMBER_HANG_OVERRIDE);

            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode
            if (gamepadEx1.wasJustPressed(B)) {
                drivetrain.setCurrentHeading(Math.PI);
            }

            drivetrain.setFieldCentricPowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    gamepadEx1.getLeftY(),
                                    -gamepadEx1.getLeftX()
                            ),
                            -gamepadEx1.getRightX()
                    )
            );

            if (gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5) {
                lift.runManual(gamepadEx2.getLeftY() * 0.2);
                lift.reset();
            } else lift.runManual((0));

            drivetrain.updatePoseEstimate();

            lift.run();
        }
    }
}
