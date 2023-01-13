

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
@TeleOp(name="TeleOpMeet2")
public class TeleOPMeet2 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack; //slideA, slideB;
    private Servo claw, pitchA, pitchB, dumbRotatingClaw;
   // private CRServo pitchA, pitchB;
    private boolean direction, togglePrecision;
    private double factor;
    private Arm Arm1;
    private Slide slides;
    boolean reverse;
    final int balls = 3;
    public double val;
    private EasyToggle toggleA = new EasyToggle("a", false, 1, false, false);
    private EasyToggle toggleB = new EasyToggle("b", false, 2, false, true);
    @Override
    public void init() {
        Arm1 = new Arm(hardwareMap);
        telemetry.addData("init start", android.R.bool::new);
        telemetry.update();
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        claw = (Servo) hardwareMap.get("CW");

        dumbRotatingClaw = (Servo) hardwareMap.get("CC");
        //slideA = (DcMotorEx) hardwareMap.dcMotor.get("SA");
        //slideB = (DcMotorEx) hardwareMap.dcMotor.get("SB");
      //  pitchA = (CRServo) hardwareMap.crservo.get("PA");
      //  pitchB = (CRServo) hardwareMap.crservo.get("PB");

       //
        // val = dumbRotatingClaw.getPosition();

        pitchA = (Servo) hardwareMap.get("PA");
        pitchB = (Servo) hardwareMap.get("PB");
        slides = new Slide(hardwareMap);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      //  pitchA.setPosition(-0.5);
      //  pitchB.setPosition(0.5);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        // rightBack.setDirection(DcMotor.Direction.REVERSE);

        //new stuff

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("init end", android.R.bool::new);
        telemetry.update();

    }
    @Override
    public void loop() {
       // telemetry.addData("loop start", android.R.bool::new);
        telemetry.update();
        toggleA.updateStart(gamepad1.a);
        toggleB.updateStart(gamepad2.b);
        //toggles precision mode if the right stick button is pressed
        if (gamepad1.left_stick_button)
            togglePrecision = true;
        else if (gamepad1.right_stick_button)
            togglePrecision = false;
        if (gamepad2.dpad_up) {
            telemetry.addData("up bumper",android.R.bool::new);
          //  telemetry.update();
            slides.setHIGH();
            Arm1.setHigh();
            //slideA.setPower(1);
            //slideB.setPower(1);
         //   slides.update(telemetry);
        }
        else if(gamepad2.dpad_left){
            telemetry.addData("left bumper",android.R.bool::new);
           // telemetry.update();
            slides.setAboveG();
            //slides.setHIGH();
        //    slides.update(telemetry);

        }
        else if (gamepad2.dpad_down) {
            telemetry.addData("low bumper",android.R.bool::new);
           // telemetry.update();
            slides.setLOW();
            //slideA.setPower(-1);
            //slideB.setPower(-1);
          //  slides.update(telemetry);
        }
        else if(gamepad2.dpad_right){
            telemetry.addData("right bumper",android.R.bool::new);
         //   telemetry.update();
            slides.setGROUND();
          //  slides.update(telemetry);
        }

        if(gamepad2.right_trigger >= 0.49){
            val+=0.01;
            dumbRotatingClaw.setPosition(val);
        }



        else if(gamepad2.left_trigger>= 0.49){
            val -=0.01;
            dumbRotatingClaw.setPosition(val);
        }




        //else{
            //slideA.setPower(0);
            //slideB.setPower(0);
        //}
      //  if(gamepad2.right_stick_button)
        //    }
        if(gamepad2.y){
            Arm1.setExtake();

        }
        else if(gamepad2.x){
            Arm1.setIntake();
        }
        Arm1.update(telemetry);
        if(gamepad2.right_bumper){
            claw.setPosition(ClawPositions.OPEN);
            //claw.setPosition(0);
        }
        else if(gamepad2.left_bumper)
            claw.setPosition(ClawPositions.CLOSED);
            //claw.setPosition(-0.7);
       // else
         //   claw.setPosition(0.4);
        /*
        if(gamepad2.right_stick_button){
            pitchA.setPosition(1);
            pitchB.setPosition(-1);
        }

         */
        /*
        int servoPos = 0
        if( gamepad2.left_stick_x>0){
            pitchA.servoPos += gamepad.gamepad1.left_stick_x * 0.2;
        }

        robot.setServo(robot.servoPos);

         */


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
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setPower(leftFrontPower * factor);
        leftBack.setPower(leftRearPower * factor);
        rightFront.setPower(rightFrontPower * factor);
        rightBack.setPower(rightRearPower * factor);

       // telemetry.addData("loop end", android.R.bool::new);
        slides.update(telemetry);
        telemetry.update();
        toggleA.updateEnd();
        toggleB.updateEnd();
    }


// heading to brazil
// also doing your mom imo
    // bruh
    // wait


}