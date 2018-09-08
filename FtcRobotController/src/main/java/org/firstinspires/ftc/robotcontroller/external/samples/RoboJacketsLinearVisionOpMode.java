package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Created by zipper on 2/4/17.
 *
 * Abstract class that contains the bulk of our code for the velocity vortex challenge.
 * All of the helper methods for our auto opmodes and teleop opmodes are neatly kept here.
 */

public abstract class RoboJacketsLinearVisionOpMode extends LinearVisionOpMode {
    private double NOT_DEPLOYED_POWER = 0;
    private double DEPLOYED_POWER = .5;
    private double CLAMP_LEFT_OPEN = .1;
    private double CLAMP_LEFT_CLOSED = .9;
    private double CLAMP_RIGHT_OPEN = .1;
    private double CLAMP_RIGHT_CLOSED = .9;
    private double GLYPH_IN = .1;
    private double GLYPH_PUSH = .9;
    private double RELIC_CLAW_UP = .1;
    private double RELIC_CLAW_DOWN = .5;
    private double RELIC_CLAW_OPEN = .9;
    private double RELIC_CLAW_CLOSED = .4;

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor glyphLift;
    private DcMotor relicExtend;
    private Servo deploy;
    private Servo clampLeft;
    private Servo clampRight;
    private Servo pushGlyph;
    private Servo relicClawPulley;
    private Servo relicClaw;
    public ElapsedTime runtime = new ElapsedTime();

    private int frameCount = 0;

    private OpenGLMatrix lastLocation = null;

    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;

    public boolean blueLeft;
    public RelicRecoveryVuMark glyphCol = RelicRecoveryVuMark.UNKNOWN;

    /**
     * Initializes all necessary components including
     * Motors
     * Sensors
     * Servos
     */
    public void initialize(boolean isAuto) throws InterruptedException {
                leftFront = hardwareMap.dcMotor.get("leftFront");
                leftBack = hardwareMap.dcMotor.get("leftBack");
                leftFront.setDirection(DcMotor.Direction.REVERSE);
                rightFront = hardwareMap.dcMotor.get("rightFront");
                rightBack = hardwareMap.dcMotor.get("rightBack");
                rightBack.setDirection(DcMotor.Direction.REVERSE);

        //        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        //        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        //        glyphLift = hardwareMap.dcMotor.get("glyphLift");
        //        relicExtend = hardwareMap.dcMotor.get("relicExtend");
        //
        //        pushGlyph = hardwareMap.servo.get("pushGlyph");
        //        clampLeft = hardwareMap.servo.get("clampLeft");
        //        clampRight = hardwareMap.servo.get("clampRight");
        deploy = hardwareMap.servo.get("deploy");
        relicClawPulley = hardwareMap.servo.get("relicClawPulley");
        relicClaw = hardwareMap.servo.get("relicClaw");

        //        pushGlyph.setPosition(GLYPH_IN);
        //        clampLeft.setPosition(CLAMP_LEFT_OPEN);
        //        clampRight.setPosition(CLAMP_RIGHT_OPEN);
        deploy.setPosition(NOT_DEPLOYED_POWER);
        relicClawPulley.setPosition(RELIC_CLAW_UP);
        relicClaw.setPosition(RELIC_CLAW_CLOSED);


        //        rightBack.setDirection(DcMotor.Direction.REVERSE);
        //        rightFront.setDirection(DcMotor.Direction.REVERSE);
        //        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        if(isAuto) {

            initOpenCV();
            sleep(4000);
            blueLeft = isBlueLeft();
            if (openCVCamera != null) {
                openCVCamera.disableView();
                openCVCamera.disconnectCamera();
            }

            if (sensors != null)
                sensors.stop();
            openCVCamera = null;
            for (Extensions extension : Extensions.values())
                if (isEnabled(extension))
                    disableExtension(extension); //disable and stop
            initVuforia();

        }
        telemetry.addData("Initialization ", "complete");
        telemetry.addData("blueLeft",blueLeft);
        telemetry.update();
    }

    /**
     * Deploys mechanisms that go outside of height
     */
    public void deployDown() {
        deploy.setPosition(DEPLOYED_POWER);
    }
    public void deployUp() {
        deploy.setPosition(NOT_DEPLOYED_POWER);
    }
    public void glyphPush() {
        pushGlyph.setPosition(GLYPH_PUSH);
    }
    public void glyphIn() {
        pushGlyph.setPosition(GLYPH_IN);
    }
    public void pulleyDown() {
        relicClawPulley.setPosition(RELIC_CLAW_DOWN);
    }
    public void pulleyUp() {
        relicClawPulley.setPosition(RELIC_CLAW_UP);
    }
    public void relicClawClose() {
        relicClaw.setPosition(RELIC_CLAW_CLOSED);
    }
    public void relicClawOpen() {
        relicClaw.setPosition(RELIC_CLAW_OPEN);
    }
    public void relicExtend(double power) {
        relicExtend.setPower(power);
    }
    public void intake(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }
    public void glyphLift(double power) {
        glyphLift.setPower(power);
    }

    public void glyphClampClose() {
        clampLeft.setPosition(CLAMP_LEFT_CLOSED);
        clampRight.setPosition(CLAMP_RIGHT_CLOSED);
    }
    public void glyphClampOpen() {
        clampLeft.setPosition(CLAMP_LEFT_OPEN);
        clampRight.setPosition(CLAMP_RIGHT_OPEN);
    }
    public void setPower(double powerLeft, double powerRight) {
        telemetry();
        leftFront.setPower(powerLeft);
        leftBack.setPower(powerLeft);
        rightFront.setPower(powerRight);
        rightBack.setPower(powerRight);
    }
    /**
     * Go forward indefinitely
     *
     * @param power Drive at this power
     * @throws InterruptedException
     */
    public void forward(double power) throws InterruptedException {
        telemetry();
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    /**
     * Turn left indefinitely
     *
     * @param power Turn at this power
     * @throws InterruptedException
     */
    public void left(double power) throws InterruptedException {
        telemetry();
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ASjtsmX/////AAAAGRIKfx5rxUpHh/PGnhhtxFhARxDTGhbXGgYJW2M7DJjZagpGX95lZcr+fiuH/OGcw0aoprFZE0yjWDGZROVrgCnWeURYO9lw6IKCGWZVRJA0AmiVfyFWOUVXLGz5LyXFvhs8iNbAF38DFn/gbuD81RGl126CNWRK+fGiDk/dJTOZspFhZqIsV6heVpjhgb+ZUI771znQlFKR1f9t08viSaiLKXiDsD+zwpiPBh4fHPyM7w8H4wwdPBq0MHDjnfmmxUDEbTMaVeLMoLZWmav70qg9eUzdJ71yH4MnBOsmZ12F0KPQ1txlip0iOVH/0U3mVGqLRrXlhhQeVtefTF2i2kB8YiJqqnGEwsADWm4vSmQ3";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }
    public RelicRecoveryVuMark doVuforia() throws InterruptedException {

        relicTrackables.activate();

        RelicRecoveryVuMark mark = RelicRecoveryVuMark.UNKNOWN;
        int leftCount = 0;
        int centerCount = 0;
        int rightCount = 0;
        sleep(250);
        for (int i = 0; i < 20; i++) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    leftCount++;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    centerCount++;
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    rightCount++;
                }
            }
            sleep(50);
        }

        if (leftCount > centerCount && leftCount > rightCount) {
            mark = RelicRecoveryVuMark.LEFT;
        } else if (centerCount > leftCount && centerCount > rightCount) {
            mark = RelicRecoveryVuMark.CENTER;
        } else if (rightCount > centerCount && rightCount > leftCount) {
            mark = RelicRecoveryVuMark.RIGHT;
        }

        telemetry.addData("Key: ", mark.toString());
        relicTrackables.deactivate();
        return mark;
    }


    /**
     * Turn right indefinitely
     *
     * @param power Turn at this power
     * @throws InterruptedException
     */
    public void right(double power) throws InterruptedException {
        telemetry();
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
    }

    /**
     * Drive based on encoder ticks
     *
     * @param leftSpeed   Speed for left motors
     * @param rightSpeed  Speed for right motors
     * @param leftCounts  Ticks for left encoders
     * @param rightCounts Ticks for right encoders
     * @param timeoutS    Timeout seconds
     * @throws InterruptedException
     */
    public void encoderDrive(double leftSpeed, double rightSpeed, double leftCounts, double rightCounts, double timeoutS) throws InterruptedException {
        zeroEncoders();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(leftSpeed);
            rightFront.setPower(rightSpeed);
            leftBack.setPower(leftSpeed);
            rightBack.setPower(rightSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                telemetry();
                if (Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts) * (7.0 / 8) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts) * (7.0 / 8) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts) * (7.0 / 8) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts) * (7.0 / 8)) {
                    leftFront.setPower(.5 * leftSpeed);
                    rightFront.setPower(.5 * rightSpeed);
                    leftBack.setPower(.5 * leftSpeed);
                    rightBack.setPower(.5 * rightSpeed);
                    telemetry.addData("speed", .5);
                } else if (Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts) * (.5) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts) * (.5) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts) * (.5) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts) * (.5)) {
                    leftFront.setPower(.75 * leftSpeed);
                    rightFront.setPower(.75 * rightSpeed);
                    leftBack.setPower(.75 * leftSpeed);
                    rightBack.setPower(.75 * rightSpeed);
                    telemetry.addData("speed", .75);
                } else {
                    telemetry.addData("speed", 1);
                }
                if (Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts)) {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    break;
                }
                waitOneFullHardwareCycle();
            }
            stopAll();

            zeroEncoders();
            //sleep(250);   // optional pause after each move
        }
    }

    /**
     * Zeroes encoders and resets them
     */
    public void zeroEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Stops all motors
     *
     * @throws InterruptedException
     */
    public void stopAll() throws InterruptedException {
        telemetry();
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * Telemetries all debugging information
     */
    public void telemetry() {
        telemetry.addData("leftFront to position", (leftFront.getTargetPosition() - leftFront.getCurrentPosition()));
        telemetry.addData("rightFront to position", (rightFront.getTargetPosition() - rightFront.getCurrentPosition()));
        telemetry.addData("leftBack to position", (leftBack.getTargetPosition() - leftBack.getCurrentPosition()));
        telemetry.addData("rightBack to position", (rightBack.getTargetPosition() - rightBack.getCurrentPosition()));
        telemetry.addData("mark",glyphCol);
        telemetry.update();
    }
    public void initOpenCV() throws InterruptedException {
        waitForVisionStart();

        this.setCamera(Cameras.PRIMARY);

        this.setFrameSize(new Size(200, 200));

        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control


        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
    }
    /**
     * Processes jewel orientations
     */
    public boolean isBlueLeft() throws InterruptedException {


        boolean blueLeft = false;
        int blueLeftCount = 0;
        int blueRightCount = 0;

        for (int i = 0; i < 20; i++) {

            if (hasNewFrame()) {
                //Get the frame
                Mat rgba = getFrameRgba();
                if (blueLeftHelper(rgba)) {
                    blueLeftCount++;
                } else {
                    blueRightCount++;
                }

                //Discard the current frame to allow for the next one to render
                discardFrame();

                //Do all of your custom frame processing here
                //For this demo, let's just add to a frame counter
                frameCount++;
            }
            else i--;

            //Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }

        if (blueLeftCount > blueRightCount) {
            blueLeft = true;
        }
        return blueLeft;
    }
    public boolean blueLeftTest() throws InterruptedException {
        boolean blueLeft = false;
        int blueLeftCount = 0;
        int blueRightCount = 0;
        if (hasNewFrame()) {
                //Get the frame
                Mat rgba = getFrameRgba();
                return blueLeftHelper(rgba);

                //Discard the current frame to allow for the next one to render

        }
        else {
            return blueLeftTest();
        }
    }
    public boolean blueLeftHelper(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Mat leftFrame = new Mat(hsvFrame, new Range(hsvFrame.rows()/2, hsvFrame.rows()), new Range(0, hsvFrame.cols()));
        Mat leftBlue = new Mat();
        Core.inRange(leftFrame, new Scalar(90, 150, 150), new Scalar(105, 255, 255), leftBlue);

        int leftCount = 0;
        for (int i = 0; i < leftBlue.rows(); i++) {
            for (int j = 0; j < leftBlue.cols(); j++) {
                if (leftBlue.get(i, j)[0] != 0) {
                    leftCount++;
                }
            }
        }

        Mat rightFrame = new Mat(hsvFrame, new Range(0, hsvFrame.rows()/2), new Range(0, hsvFrame.cols()));
        Mat rightBlue = new Mat();
        Core.inRange(rightFrame, new Scalar(90, 150, 150), new Scalar(105, 255, 255), rightBlue);

        int rightCount = 0;
        for (int i = 0; i < rightBlue.rows(); i++) {
            for (int j = 0; j < rightBlue.cols(); j++) {
                if (rightBlue.get(i, j)[0] != 0) {
                    rightCount++;
                }
            }
        }

        return leftCount > rightCount;
    }

}