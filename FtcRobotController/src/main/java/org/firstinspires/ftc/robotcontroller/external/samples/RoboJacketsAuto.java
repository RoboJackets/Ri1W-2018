package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by zipper on 9/9/17.
 */
@Autonomous
public class RoboJacketsAuto extends RoboJacketsLinearVisionOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true);

        //Key detection from pictograph

        waitForStart();
        deployDown();
        sleep(2000);
        if(!blueLeft) {
            encoderDrive(-.3, .3, 200, 200, 3);
            sleep(500);
            encoderDrive(.3, -.3, 200, 200, 3);
        }
        else {
            encoderDrive(.3, -.3, 200, 200, 3);
            sleep(500);
            encoderDrive(-.3, .3, 200, 200, 3);
        }
        deployUp();
        sleep(2000);
        //encoderDrive(-.3,-.3,650,650,3);
        encoderDrive(.3, -.3, 200, 200, 3);
        glyphCol = doVuforia();
        sleep(2000);
        encoderDrive(-.3, .3, 200, 200, 3);
        sleep(2000);
        telemetry.addData("mark",glyphCol);
        if(glyphCol==RelicRecoveryVuMark.RIGHT) {
            encoderDrive(-.3,-.3,2100,2100,5);
        }
        else if(glyphCol==RelicRecoveryVuMark.CENTER) {
            encoderDrive(-.3,-.3,2600,2600,5);
        }
        else if(glyphCol==RelicRecoveryVuMark.LEFT) {
            encoderDrive(-.3,-.3,3100,3100,5);
        }
        else {
            encoderDrive(-.3,-.3,2600,2600,5);
        }
        encoderDrive(-.3,.3,1500,1500,5);
        encoderDrive(-.3,-.3,2000,2000,5);
    }
}

