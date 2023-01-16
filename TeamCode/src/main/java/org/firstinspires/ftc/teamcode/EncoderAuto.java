package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

@Autonomous(name = "EncoderAuto")
public class EncoderAuto extends LinearOpMode {
    private DcMotorEx front_left;
    private DcMotorEx front_right;
    private DcMotorEx back_left;
    private DcMotorEx back_right;
    //private DcMotorEx sliderA, sliderB;
    private Servo claw, pitchA, pitchB;
    private Arm Arm1;
    private Slide slides;
    private AprilTagAutonomousInitDetectionExample CVDetect;

    private int leftPos;
    private int rightPos;

    /*public void initialize(){

    }
    */
    @Override
    public void runOpMode(){
        //initialize();
        slides = new Slide(hardwareMap);
        Arm1 = new Arm(hardwareMap);
        claw = hardwareMap.servo.get("CW");
        CVDetect = new AprilTagAutonomousInitDetectionExample();
        //CVDetect.runOpMode();
        front_left = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        front_right = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        back_left = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        back_right = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setDirection(DcMotorEx.Direction.FORWARD);
        front_right.setDirection(DcMotorEx.Direction.REVERSE);
        back_left.setDirection(DcMotorEx.Direction.FORWARD);
        back_right.setDirection(DcMotorEx.Direction.REVERSE);

        leftPos = 0;
        rightPos = 0;


        waitForStart();

        slides.setHIGH();
        while(opModeIsActive() && !isStopRequested()){
            slides.update(telemetry);
        }
        drive(0.5,1000,1000);
        turnR(0.5,1000, 1000);

        //slides.setMIDDLE();
        /*
        openClaw();
        sleep(1000);
        closeClaw();
        drive(0.5, 1500,1500);
        //sleep(300);
        turnR(0.5, 750, 750);
        drive(0.5, 950,950);
        //sleep(1000);
        //sleep(1000);
        // sliderA.setPower(0.5);
        //sliderB.setPower(0.5);
        //slides.setHIGH(); //set slides middle
        //sleep(1500);
        sleep(1000);
        slides.setHIGH();
        sleep(1500);
        //drive(0.5,-150,-150);

        slides.update(telemetry);
        Arm1.update(telemetry);
        //telemetry.update();
        // CVDetect.doStuff();  // makes it work

         */


    }

    //@Override
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

        //while(opModeIsActive() && front_left.isBusy() && front_right.isBusy()&& back_left.isBusy() && back_right.isBusy()){
          //  idle();
        //}
    }

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

    public void liftC(){ slides.setAboveG();}

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



}