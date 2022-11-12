package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name="TeleOPOfficial")
public class TeleOPTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, slideA, slideB, clawAngle;
    private Servo claw, pitchA, pitchB;
    private boolean direction, togglePrecision;
    private double factor;
    boolean reverse;
    final int balls = 3;
    private int HIGH,MID,LOW;
    EasyToggle toggleA = new EasyToggle("a", false, 1, false, false);
  //  EasyToggle toggleB = new EasyToggle("b", false, 1, false, false);

    @Override
    public void init() {
        telemetry.addData("init start", android.R.bool::new);
        telemetry.update();
        HIGH = 1000;
        MID = 500;
        LOW = 100;
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        slideA = (DcMotorEx) hardwareMap.dcMotor.get("SA");
        slideB = (DcMotorEx) hardwareMap.dcMotor.get("SB");
        clawAngle = (DcMotorEx) hardwareMap.dcMotor.get("CA");
        //claw = (Servo) hardwareMap.get("CW");
        claw = (Servo) hardwareMap.servo.get("CW");
        pitchA = (Servo) hardwareMap.get("PA");
        pitchB = (Servo) hardwareMap.get("PB");
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        // rightBack.setDirection(DcMotor.Direction.REVERSE);

        //new stuff

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        slideA.setDirection(DcMotor.Direction.FORWARD);
        slideB.setDirection(DcMotor.Direction.FORWARD);
        clawAngle.setDirection(DcMotor.Direction.FORWARD);



        telemetry.addData("init end", android.R.bool::new);
        telemetry.update();

    }
    private void slide(double speed, int Target){

        slideA.setTargetPosition(Target);
        slideB.setTargetPosition(Target);

        slideA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideA.setPower(speed);
        slideB.setPower(speed);
    }
    @Override
    public void loop() {
        //telemetry.addData("loop start", android.R.bool::new);
       // telemetry.update();
        toggleA.updateStart(gamepad1.a);
       // toggleB.updateStart(gamepad2.b);

        //toggles precision mode if the right stick button is pressed
        if (gamepad1.left_stick_button)
            togglePrecision = true;
        else if (gamepad1.right_stick_button)
            togglePrecision = false;
        if (gamepad2.dpad_up) {
            slideA.setPower(1);
            slideB.setPower(1);
           // slide(0.25,HIGH); //highest SET SOLUTION
        }
        /* SET SOLUTION
        else if(gamepad2.dpad_left){
           slide(s0.25,MID);
        }

         */


        else if (gamepad2.dpad_down) {
            slideA.setPower(-1);
            slideB.setPower(-1);
         //   slide(0.25,LOW); #SET SOLUTION
        }
        /* SET SOLUTION
        else if(gamepad.dpad_right){
            slide(0.25,0);
        }
         */
        else{
            slideA.setPower(0);
            slideB.setPower(0);
        }
        if(gamepad2.left_bumper){
            claw.setPosition(1);
            telemetry.addData("L bumper TESTO1",android.R.bool::new);
            telemetry.update();
        }

        if(gamepad2.right_bumper) {
            claw.setPosition(0);
            telemetry.addData("R bumper TESTO2", android.R.bool::new);
            telemetry.update();
        }
        /*
        else{
            claw.setPosition(0);
        }

         */


        if(gamepad2.x){
            clawAngle.setPower(0.5);
            //clawAngle.setTargetPosition(50);

        }
        else if(gamepad2.y){
            clawAngle.setPower(-0.5);
            //clawAngle.setTargetPosition(-50);
        }
        else{
            clawAngle.setPower(0);
        }


        //sets the factor multiplied to the power of the motors
        factor = togglePrecision ? .3 : 1.5; //the power is 1/5th of its normal value while in precision mode

        // Do not mess with this, if it works, it works
        double x = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double stickAngle = Math.atan2(direction ? -gamepad1.left_stick_y : gamepad1.left_stick_y, direction ? gamepad1.left_stick_x : -gamepad1.left_stick_x); // desired robot angle from the angle of stick
        double powerAngle = stickAngle - (Math.PI / 4); // conversion for correct power values
        double rightX = gamepad1.right_stick_x; // right stick x axis controls turning
        final double leftFrontPower = Range.clip(x * Math.cos(powerAngle) - rightX, -1.0, 1.0);
        final double leftRearPower = Range.clip(x * Math.sin(powerAngle) - rightX, -1.0, 1.0);
        final double rightFrontPower = Range.clip(x * Math.sin(powerAngle) + rightX, -1.0, 1.0);
        final double rightRearPower = Range.clip(x * Math.cos(powerAngle) + rightX, -1.0, 1.0);


        //CHANGED LEFT FRONT TO REVERSE
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setPower(leftFrontPower * factor);
        leftBack.setPower(leftRearPower * factor);
        rightFront.setPower(rightFrontPower * factor);
        rightBack.setPower(rightRearPower * factor);

        telemetry.addData("loop end", android.R.bool::new);
        telemetry.update();
        toggleA.updateEnd();
       // toggleB.updateEnd();
    }



// heading to brazil
// also doing your mom imo
    // bruh
    // wait


}