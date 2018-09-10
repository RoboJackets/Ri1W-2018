package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
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
 * Created by jviszlai on 9/8/18.
 */
public abstract class RoboJacketsVisionOpMode extends LinearVisionOpMode{


    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    private DcMotor intake;

    private DcMotor elevator1;
    private DcMotor elevator2;

    private Servo servo;

    private Scalar lowerBoundYellow = new Scalar(18, 200, 200);
    private Scalar upperBoundYellow = new Scalar(25, 255, 255);

    public ElapsedTime runtime = new ElapsedTime();

    private int frameCount = 0;

    private OpenGLMatrix lastLocation = null;

    private VuforiaLocalizer vuforia;

    private double[] position = new double[]{0, 0, 0, 0, 0, 0};;

    public int goldPosition;

    /**
     * Initializes all necessary components including
     * Motors
     * Sensors
     * Servos
     */
    public void initialize(boolean isAuto) throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("3");
        leftBack = hardwareMap.dcMotor.get("2");
        rightFront = hardwareMap.dcMotor.get("0");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack = hardwareMap.dcMotor.get("1");
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.dcMotor.get("intake");

        elevator1 = hardwareMap.dcMotor.get("elevator1");
        elevator2 = hardwareMap.dcMotor.get("elevator2");

        servo = hardwareMap.servo.get("servo");

        if(isAuto) {

            initOpenCV();
            sleep(4000);
            if (hasNewFrame()) {
                goldPosition = getGoldPosition();
            }
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
            position = findPosition();
        }
        telemetry.addData("Initialization ", "complete");
        telemetry.addData("Gold Position", goldPosition);

        telemetry.addData("X", position[0]);
        telemetry.addData("Y", position[1]);
        telemetry.addData("Z", position[2]);
        telemetry.addData("rX", position[3]);
        telemetry.addData("rY", position[4]);
        telemetry.addData("rZ", position[5]);


        telemetry.update();
    }


    public void setServo(double position) {
        servo.setPosition(position);
    }

    /**
     * Set elevator power
     * @param power for elevator
     */
    public void elevatorPower(double power) {
        elevator1.setPower(power);
        elevator2.setPower(power);
    }


    /**
     * Run intake motor in reverse for outake
     */

    public void outtake(double power) {
        intake.setPower(-power);
    }

    /**
     * Run intake motor
     */
    public void intake(double power) {
        intake.setPower(power);
    }


    /**
     * Set drive motor power
     * @param powerLeft left motors power
     * @param powerRight right motors power
     */
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
     * Find position to image target
     * @return position of robot with respect to image target
     */
    public double[] findPosition() {
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                    double[] coords = new double[6];
                    coords[0] = tX;
                    coords[1] = tY;
                    coords[2] = tZ;
                    coords[3] = rX;
                    coords[4] = rY;
                    coords[5] = rZ;
                    return coords;
                }

            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
        return new double[]{0, 0, 0, 0, 0, 0};
    }

    /**
     * Format transformation matrix for pose estimate
     * @param transformationMatrix matrix to format
     * @return formatted matrix
     */
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    /**
     * Initialize Vuforia
     */
    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ae6AwNj/////AAABmb237TMSwkT3r7lIX9cPOSN6CShC2Dhjbn5HEXZuR/9fEimW0fSd9J520pzgu9IeDP88wfoxAgd1V1x+q+xK0yMyhfGUe5Y5nihGryy768Guz16x9EFBMKVII275+vUVCo4ZTmeFOBG2f2tL1wlAZQ+yltUzu1F+ju/DyVJqZVGAU6XgrOsAYLCUbtLtg8zIG2KGX4H5nbMmU0bv11WY61thCj1k+27Do4D8hOvr5MixSLA99FdJBwV+FgPkLZUN6INZwzMk68GmZ5fhjBgw0ZwjRdmYyjyLOb+QpGcCq82O9Rvc39HIkjmFDlxzf8Q/TzDiG1/wNoiUQ2BHwyrsyubT1UPynBKPT1WkXilSt3rP";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

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
     * Finds most consistent position over 20 frames.
     * @return Index: 0, 1, or 2. Referring to gold in left, center, or right, respectively.
     * @throws InterruptedException
     */

    public int getGoldPosition() throws InterruptedException {
        int yellowLeftCount = 0;
        int yellowCenterCount = 0;
        int yellowRightCount = 0;

        for (int i = 0; i < 20; i++) {

            if (hasNewFrame()) {
                //Get the frame
                Mat rgba = getFrameRgba();
                int position = getGoldPositionSingleFrame(rgba);
                if (position == 0) {
                    yellowLeftCount++;
                } else if (position == 1) {
                    yellowCenterCount++;
                } else {
                    yellowRightCount++;
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

        int max = Math.max(yellowLeftCount, Math.max(yellowCenterCount, yellowRightCount));
        if (max == yellowLeftCount) {
            return 0;
        }
        if (max == yellowCenterCount) {
            return 1;
        }
        return 2;
    }


    /**
     *
     * @param frame, Input current camera frame
     * @return Index: 0, 1, or 2. Referring to gold in left, center, or right, respectively.
     */

    public int getGoldPositionSingleFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
        Mat left = new Mat(hsvFrame, new Range(0, hsvFrame.rows()), new Range(0, hsvFrame.cols() / 3));
        Mat center = new Mat(hsvFrame, new Range(0, hsvFrame.rows()), new Range(hsvFrame.cols() / 3, 2 * hsvFrame.cols() / 3));
        Mat right = new Mat(hsvFrame, new Range(0, hsvFrame.rows()), new Range(2 * hsvFrame.cols() / 3, hsvFrame.cols()));

        Mat yellowFilteredL = new Mat();
        Core.inRange(left, lowerBoundYellow, upperBoundYellow, yellowFilteredL);
        int leftCount = 0;
        for (int i = 0; i < yellowFilteredL.rows(); i++) {
            for (int j = 0; j < yellowFilteredL.cols(); j++) {
                if (yellowFilteredL.get(i, j)[0] != 0) {
                    leftCount++;
                }
            }
        }

        Mat yellowFilteredC = new Mat();
        Core.inRange(center, lowerBoundYellow, upperBoundYellow, yellowFilteredC);
        int centerCount = 0;
        for (int i = 0; i < yellowFilteredC.rows(); i++) {
            for (int j = 0; j < yellowFilteredC.cols(); j++) {
                if (yellowFilteredC.get(i, j)[0] != 0) {
                    centerCount++;
                }
            }
        }

        Mat yellowFilteredR = new Mat();
        Core.inRange(right, lowerBoundYellow, upperBoundYellow, yellowFilteredR);
        int rightCount = 0;
        for (int i = 0; i < yellowFilteredR.rows(); i++) {
            for (int j = 0; j < yellowFilteredR.cols(); j++) {
                if (yellowFilteredR.get(i, j)[0] != 0) {
                    rightCount++;
                }
            }
        }

        int max = Math.max(leftCount, Math.max(centerCount, rightCount));
        if (max == leftCount) {
            return 0;
        }
        if (max == centerCount) {
            return 1;
        }
        return 2;
    }


}
