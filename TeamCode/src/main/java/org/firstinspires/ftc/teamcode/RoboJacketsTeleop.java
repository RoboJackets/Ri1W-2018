package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by jviszlai on 9/8/18.
 */

@TeleOp
public class RoboJacketsTeleop extends RoboJacketsVisionOpMode {

    public void runOpMode() throws InterruptedException {
        initialize(false);
        waitForStart();
        while(opModeIsActive()) {
            teleop();
        }
    }

    public void teleop() throws InterruptedException {
        setPower(gamepad1.left_stick_y, gamepad1.right_stick_y);
        if (gamepad1.right_trigger > 0.1) {
            intake(gamepad1.right_trigger);

        } else if (gamepad1.left_trigger > 0.1) {
            outtake(gamepad1.left_trigger);
        } else {
            intake(0);
        }
        elevatorPower(gamepad2.left_stick_y);
        setServo(gamepad2.right_stick_y < 0 ? -gamepad2.right_stick_y : gamepad2.right_stick_y);
    }
}
