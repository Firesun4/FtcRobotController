package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "EncoderAuto")
public class EncoderAuto extends AprilTagAutonomousInitDetectionExample{
    private DcMotorEx front_left;
    private DcMotorEx front_right;
    private DcMotorEx back_left;
    private DcMotorEx back_right;

    private int leftPos;
    private int rightPos;
    @Override
    public void runOpMode(){
        super.runOpMode();
        front_left = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        front_right = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        back_left = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        back_right = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setDirection(DcMotorEx.Direction.REVERSE);
        front_right.setDirection(DcMotorEx.Direction.REVERSE);
        back_left.setDirection(DcMotorEx.Direction.REVERSE);
        back_right.setDirection(DcMotorEx.Direction.REVERSE);

        leftPos = 0;
        rightPos = 0;


        waitForStart();
        drive(0.25, 1000, 1000);
        drive(0.25, 1000,-1000);
    }

    //@Override
    private void drive(double speed, int leftTarget, int rightTarget){
        leftPos += leftTarget;
        rightPos += rightTarget;

        front_left.setTargetPosition(leftPos);
        back_left.setTargetPosition(leftPos);

        front_right.setTargetPosition(rightPos);
        back_right.setTargetPosition(rightPos);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(speed);
        right.setPower(speed);

        while(opModeIsActive() && left.isBusy() && right.isBusy()){
            idle();
        }

        //leftFront.setVelocity(10, AngleUnit.RADIANS);
    }

}
