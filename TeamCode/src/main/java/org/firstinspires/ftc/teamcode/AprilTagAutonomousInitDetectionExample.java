/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name = "selfDestruct")
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    private DcMotorEx front_left;
    private DcMotorEx front_right;
    private DcMotorEx back_left;
    private DcMotorEx back_right;
    private DcMotorEx clawAngle;
    private Slide slides;
    private Arm Arm1;
    private ElapsedTime timer;
    //private Servo;

    private int leftPos;
    private int rightPos;
    private Servo claw;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        claw = hardwareMap.servo.get("CW");
     //   pitchA = hardwareMap.servo.get("PA");
      //  pitchB = hardwareMap.servoq.get("PB");


    //    pitchA.setPosition(0.8);
     //   pitchB.setPosition(0.2);
     //   sleep(500);
//        pitchB.setPosition(0.2);
        claw.setPosition(ClawPositions.CLOSED);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        /*
        clawAngle = (DcMotorEx) hardwareMap.dcMotor.get("CA");
        clawAngle.setTargetPosition(1);
        clawAngle.setPower(-0.25);
        clawAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            //sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        front_left = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        front_right = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        back_left = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        back_right = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        slides = new Slide(hardwareMap);
        Arm1 = new Arm(hardwareMap);
       // pitchA = hardwareMap.servo.get("PA");
       // pitchB = hardwareMap.servo.get("PB");


        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //old
        /*
        front_left.setDirection(DcMotorEx.Direction.REVERSE);
        front_right.setDirection(DcMotorEx.Direction.FORWARD);
        back_left.setDirection(DcMotorEx.Direction.FORWARD);
        back_right.setDirection(DcMotorEx.Direction.REVERSE);
         */

        front_left.setDirection(DcMotorEx.Direction.FORWARD);
        front_right.setDirection(DcMotorEx.Direction.REVERSE);
        back_left.setDirection(DcMotorEx.Direction.FORWARD);
        back_right.setDirection(DcMotorEx.Direction.REVERSE);

        //Arm1.setHigh();
        //slides.setLOW();

       // slides.update(telemetry);
       // Arm1.update(telemetry);
       // telemetry.update();
        leftPos = 0;
        rightPos = 0;

        /* Actually do something useful */
        int turnVal = 1100;
        int forward = 2100 ;

        //Auto slides and claw

        //slides.setMIDDLE();


        //drive(0.5,1500, 1500)
        /*drive(0.5, 2200,2200);
        sleep(2000);

        turnR(0.5, 10, 0);
        sleep(110000000);
        openClaw();
        sleep(2000);
        slides.setAboveG();
        sleep(2000);
        drive(0.5, 500,500);
        sleep(2000);
        closeClaw();
        sleep(2000);
        turnR(0.5,1500,1500);
        sleep(2000);
        drive(0.5,750,750);
        sleep(2000);
        turnL(0.5,100,100);
        sleep(2000);
        slides.setMIDDLE();
        sleep(2000);
        drive(0.5,50,50);
        sleep(2000);
        openClaw();
        sleep(2000);
        //slides.setAboveG();
        


        //sleep(1500);
        /*
        //sleep(1000);
        // sliderA.setPower(0.5);
        //sliderB.setPower(0.5);
        drive(0.5, 600,600);
        //turnR(0.5, 1250, 1250);
        //slides.setHIGH(); //set slides middle
        sleep(1500);
        //turnR(0.5,750,750);
        //slides.setHIGH();
       // sleep(500);
        //drive(0.5,-150,-150);

       // slides.update(telemetry);
        //Arm1.update(telemetry);

         */



        if(tagOfInterest == null || tagOfInterest.id == LEFT){
           // pitchA.setPosition(0.2);
           //pitchB.setPosition(0.8);
            claw.setPosition(ClawPositions.CLOSED);
            //timer = new ElapsedTime();
            drive(0.5,50,50);
            drive(0.5, 1175, 1175);
            timer = new ElapsedTime();
            slides.setHIGH();
            sleep(1000);


            while(opModeIsActive() && !isStopRequested() && timer.milliseconds() < 4000){
                slides.update(telemetry);
            }
            drive(0.5,-655,655);
            //drive(0.5,(forward/2)+75,(forward/2)+75);



            //drive(0.5,-30,30);
            sleep(1000);
            claw.setPosition(ClawPositions.OPEN);
            sleep(1000);
            drive(0.5, -50, -50);
            drive(0.5, -585, 585);
            drive(0.5, 50, 50);
            timer = new ElapsedTime();
            slides.setGROUND();
            while(opModeIsActive() && !isStopRequested() && timer.milliseconds() < 4000){
                slides.update(telemetry);
            }
            sleep(1000);
            drive(0.1,1,1);




            /*
            drive(0.5, 50,50);
            drive(0.5, -turnVal,turnVal);
            drive(0.5,forward/2,forward/2);
            drive(0.5,-50, 50);

             */
            //    drive(0.5, 675,-675);
            //   drive(0.5,forward,forward);


        }else if(tagOfInterest.id == MIDDLE){
            /*
            drive(0.5, 85,85);
            drive(0.5, -turnVal,turnVal);
            drive(0.5,forward,forward);
            drive(0.5, 700,-700);
            drive(0.5,1850,1850);
            sleep(500);
            drive(0.5, 770,-770);
            drive(0.5,150,150);

             */

           // pitchA.setPosition(0.3);
           // pitchB.setPosition(0.7);
            claw.setPosition(ClawPositions.CLOSED);
            //timer = new ElapsedTime();
            drive(0.5,50,50);
            drive(0.5, 1175, 1175);
            timer = new ElapsedTime();
            slides.setHIGH();
            sleep(1000);


            while(opModeIsActive() && !isStopRequested() && timer.milliseconds() < 4000){
                slides.update(telemetry);
            }
            drive(0.5,-655,655);
            //drive(0.5,(forward/2)+75,(forward/2)+75);



            //drive(0.5,-30,30);
            sleep(500);
            claw.setPosition(ClawPositions.OPEN);
            sleep(1000);
            drive(0.5, -50, -50);
            drive(0.5, 600, -600);
            drive(0.5, 50, 50);
            timer = new ElapsedTime();
            slides.setGROUND();
            while(opModeIsActive() && !isStopRequested() && timer.milliseconds() < 4000){
                slides.update(telemetry);
            }
            sleep(1000);
            drive(0.1,1,1);

        }

        else {
          //  pitchA.setPosition(0.3);
           // pitchB.setPosition(0.7);
            claw.setPosition(ClawPositions.CLOSED);
            //timer = new ElapsedTime();
            drive(0.5,50,50);
            drive(0.5, 1175, 1175);
            timer = new ElapsedTime();
            slides.setHIGH();
            sleep(1000);


            while(opModeIsActive() && !isStopRequested() && timer.milliseconds() < 4000){
                slides.update(telemetry);
            }
            drive(0.5,-655,655);
            //drive(0.5,(forward/2)+75,(forward/2)+75);



            //drive(0.5,-30,30);
            sleep(1000);
            claw.setPosition(ClawPositions.OPEN);
            sleep(1000);
            drive(0.5, -50, -50);
            drive(0.5, 1800, -1800);
            drive(0.5, 50, 50);
            timer = new ElapsedTime();
            slides.setGROUND();
            while(opModeIsActive() && !isStopRequested() && timer.milliseconds() < 4000){
                slides.update(telemetry);
            }
            sleep(1000);
            drive(0.1,1,1);


            /*
            drive(0.5, 55, 55);
            drive(0.5, turnVal, -turnVal);
            drive(0.5, forward/2, forward/2);
            drive(0.5, 50, -50);
            slides.setAboveG();

             */

    //  drive(0.5, -675,675);
    // drive(0.5,forward,forward);
}





       // slides.setGROUND();


        //waitForStart();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    private void drive(double speed, int leftTarget, int rightTarget){
        leftPos += leftTarget;
        rightPos += rightTarget;

        front_left.setTargetPosition(-leftPos);
        back_left.setTargetPosition(leftPos);

        front_right.setTargetPosition(-rightPos);
        back_right.setTargetPosition(rightPos);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(speed);
        front_right.setPower(speed);
        back_left.setPower(speed);
        back_right.setPower(speed);

        while(opModeIsActive() && front_left.isBusy() && front_right.isBusy()&& back_left.isBusy() && back_right.isBusy()){
            idle();
        }

        //leftFront.setVelocity(10, AngleUnit.RADIANS);
    }
    /*
    private void turnR(double speed, int leftTarget, int rightTarget){

        //Drive
        leftPos += leftTarget;
        rightPos += rightTarget;

        front_left.setTargetPosition(-leftPos);
        back_left.setTargetPosition(-leftPos);

        front_right.setTargetPosition(rightPos);
        back_right.setTargetPosition(rightPos);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(speed);
        front_right.setPower(speed);
        back_left.setPower(speed);
        back_right.setPower(speed);

        while(opModeIsActive() && front_left.isBusy() && front_right.isBusy()&& back_left.isBusy() && back_right.isBusy()){
            idle();
        }
    }

     */
    /*
    private void turnL(double speed, int leftTarget, int rightTarget){

        //Drive
        leftPos += leftTarget;
        rightPos += rightTarget;

        front_left.setTargetPosition(leftPos);
        back_left.setTargetPosition(leftPos);

        front_right.setTargetPosition(-rightPos);
        back_right.setTargetPosition(-rightPos);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(speed);
        front_right.setPower(speed);
        back_left.setPower(speed);
        back_right.setPower(speed);

        while(opModeIsActive() && front_left.isBusy() && front_right.isBusy()&& back_left.isBusy() && back_right.isBusy()){
            idle();
        }
    }



    public void lift(){
        slides.setMIDDLE();
    }

    public void openClaw(){
        claw.setPosition(ClawPositions.OPEN);
    }

    public void closeClaw(){
        claw.setPosition(ClawPositions.CLOSED);
    }

    public void dropLift(){
        //change maybe to aboveG
        slides.setGROUND();
    }

     */

}