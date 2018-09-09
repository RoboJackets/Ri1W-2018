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
    }
}
