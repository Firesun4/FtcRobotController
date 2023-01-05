package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOPOfficial")
public class TeleOPTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, slideA, slideB, clawAngle;
    private Servo claw, pitchA, pitchB, clawServoUn;
    private boolean direction, togglePrecision;
    private double factor;
    private EasyToggle toggleA = new EasyToggle("a", false, 1, false, false);
    boolean reverse;
    private EasyToggle toggleB = new EasyToggle("b", false, 2, false, true);

    @Override
    public void init() {
        telemetry.addData("init start", android.R.bool::new);
        telemetry.update();
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        slideA = (DcMotorEx) hardwareMap.dcMotor.get("SA");
        slideB = (DcMotorEx) hardwareMap.dcMotor.get("SB");
        clawAngle = (DcMotorEx) hardwareMap.dcMotor.get("CA");
        claw = hardwareMap.servo.get("CW");
        pitchA = hardwareMap.servo.get("PA");
        pitchB = hardwareMap.servo.get("PB");
        clawServoUn = hardwareMap.servo.get("CC");
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

        telemetry.addData("Claw Port:", (claw.getPortNumber()));
        telemetry.addData("PA Port:", (pitchA.getPortNumber()));
        claw.setPosition(0);
        //clawServoUn.setPosition(0);

        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        // rightBack.setDirection(DcMotor.Direction.REVERSE);

        //new stuff

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        slideA.setDirection(DcMotor.Direction.REVERSE);
        slideB.setDirection(DcMotor.Direction.FORWARD);
        clawAngle.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("init end", android.R.bool::new);
        telemetry.update();

    }

    @Override
    public void loop() {

        //telemetry.addData("loop start", android.R.bool::new);
        //telemetry.update();
        toggleA.updateStart(gamepad1.a);
        toggleB.updateStart(gamepad2.b);

        //clawServoUn.setPosition(0);

        //toggles precision mode if the right stick button is pressed
        if (gamepad1.left_stick_button)
            togglePrecision = true;
        else if (gamepad1.right_stick_button)
            togglePrecision = false;
        if (gamepad2.dpad_up) {
            slideA.setPower(0.7);
            slideB.setPower(0.7);
        } else if (gamepad2.dpad_down) {
            slideA.setPower(-0.5);
            slideB.setPower(-0.5);
        } else {
            slideA.setPower(0);
            slideB.setPower(0);
        }
        /*
        int fr = 0;
        if(gamepad1.right_bumper){
            clawServoUn.setPosition(0.01);
        }
        else if(gamepad1.left_bumper){
            clawServoUn.setPosition(0.01);
        }

         */

        if (gamepad2.left_bumper) {
            claw.setPosition(ClawPositions.OPEN);
//            clawNew.setPosition(0.50);
            telemetry.addData("right trigger call: ", gamepad1.right_bumper);
            telemetry.update();
        } else if (gamepad2.right_bumper)
            claw.setPosition(ClawPositions.CLOSED);
        telemetry.addData("left trigger call: ", gamepad2.left_trigger);
        telemetry.update();

        if (gamepad2.x) {
            clawAngle.setPower(0.75);

        } else if (gamepad2.y) {
            clawAngle.setPower(-0.75);
        } else {
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
        leftBack.setDirection(DcMotor.Direction.REVERSE); //changged 30
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD); // changed 30
        leftFront.setPower(leftFrontPower * factor);
        leftBack.setPower(leftRearPower * factor);
        rightFront.setPower(rightFrontPower * factor);
        rightBack.setPower(rightRearPower * factor);

        //telemetry.addData("loop end", android.R.bool::new);
        // telemetry.update();
        toggleA.updateEnd();
        toggleB.updateEnd();
    }


// heading to brazil
// also doing your mom imo
    // bruh
    // wait


}