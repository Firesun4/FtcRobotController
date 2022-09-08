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
    private DcMotorEx left;
    private DcMotorEx right;

    private int leftPos;
    private int rightPos;
    @Override
    public void runOpMode(){
        left = (DcMotorEx) hardwareMap.dcMotor.get("leftMotor");
        right = (DcMotorEx) hardwareMap.dcMotor.get("rightMotor");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setDirection(DcMotorEx.Direction.REVERSE);
        right.setDirection(DcMotorEx.Direction.REVERSE);

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

        left.setTargetPosition(leftPos);
        right.setTargetPosition(rightPos);

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
