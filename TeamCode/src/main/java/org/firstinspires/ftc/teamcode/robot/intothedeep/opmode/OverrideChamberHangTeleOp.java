package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

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

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Lift;

@TeleOp(group = "24064 Main")
public class OverrideChamberHangTeleOp extends LinearOpMode {
    GamepadEx gamepadEx1;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx1 = new GamepadEx(gamepad1);

        Pose2d endPose = Common.AUTO_END_POSE;
        if (endPose != null) {
            robot.drivetrain.setCurrentHeading(endPose.heading.toDouble() - Common.FORWARD);
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.readSensors();
            gamepadEx1.readButtons();

            robot.lift.setTargetTicks(Lift.Ticks.CHAMBER_HANG_OVERRIDE);

            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode
            double x = gamepadEx1.getRightX();
            if (gamepadEx1.isDown(RIGHT_STICK_BUTTON)) {
                double y = gamepadEx1.getRightY();
                if (hypot(x, y) >= 0.8) robot.drivetrain.setCurrentHeading(atan2(y, x));
                x = 0;
            }

            robot.drivetrain.setFieldCentricPowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    gamepadEx1.getLeftY(),
                                    -gamepadEx1.getLeftX()
                            ),
                            -gamepadEx1.getRightX()
                    )
            );

            robot.drivetrain.updatePoseEstimate();
            robot.run();
        }
    }
}
