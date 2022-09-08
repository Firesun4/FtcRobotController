package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "EncoderAuto")
public class EncoderAuto extends LinearOpMode{
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    private double leftFrontPos;
    private double leftBackPos;
    private double rightFrontPos;
    private double rightBackPos;

    public void runOpMode(){
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);



        leftFrontPos = 0.03;
        leftBackPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0.03  ;

        waitForStart();
    }

    //@Override
    private void drive(double speed, int leftFrontTarget, int leftBackTarget, int rightFrontTarget, int rightBackTarget){
        leftFrontPos += leftFrontTarget;
        leftBackPos += leftBackTarget;
        rightFrontPos += rightFrontTarget;
        rightBackPos += rightBackTarget;

        //leftFront.setVelocity(10, AngleUnit.RADIANS);
    }
}
