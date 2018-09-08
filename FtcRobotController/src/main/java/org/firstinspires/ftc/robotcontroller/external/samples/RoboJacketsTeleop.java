package org.firstinspires.ftc.robotcontroller.external.samples;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by zipper on 9/9/17.
 */
@TeleOp
public class RoboJacketsTeleop extends RoboJacketsLinearVisionOpMode {
    private boolean intakeToggle = false;
    private boolean relicClawToggle = false;
    private boolean relicPulleyToggle = false;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(false);
        waitForStart();
        while(opModeIsActive()){
            teleop();
            if(gamepad1.b) {
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    glyphClampClose();
                    glyphLift(1);
                    teleop();
                }
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    glyphPush();
                    teleop();
                }
                glyphIn();
                glyphClampOpen();
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    glyphLift(-1);
                    teleop();
                }
            }
        }
    }

    public void teleop() throws InterruptedException {
        setPower(gamepad1.left_stick_y,gamepad1.right_stick_y);
        //if(intakeToggle) intake(1);
        //else intake(0);
        /*if(relicClawToggle) relicClawClose();
        else relicClawOpen();
        if(relicPulleyToggle) pulleyUp();
        else pulleyDown();
        if(gamepad1.a) {
            intakeToggle = !intakeToggle;
            sleep(150);
        }
        if(gamepad1.x) {
            relicClawToggle = !relicClawToggle;
            sleep(150);
        }
        if(gamepad1.y) {
            relicPulleyToggle = !relicPulleyToggle;
            sleep(150);
        }
        //telemetry();
        */
    }
}
