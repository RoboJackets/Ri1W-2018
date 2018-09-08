package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by zipper on 9/10/17.
 */
@Autonomous
public class RoboJacketsVisionTest extends RoboJacketsLinearVisionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(false);
        initOpenCV();
        sleep(3000);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("blueLeft",blueLeftTest());
            telemetry.update();
        }
    }
}
