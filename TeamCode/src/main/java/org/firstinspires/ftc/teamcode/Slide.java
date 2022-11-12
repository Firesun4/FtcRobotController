package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide {
    private PIDFController armPIDF;
    private Motor LMotor;
    private Motor RMotor;

    // Declaring and Initializing PIDF values

    //CHANGE THESE
    public static double armKp = 0.00097; // change first and start at 0
    public static double armKi = 0.000001; // if you always have a equal amount of error make higher
    public static double armKd = 0; // only used if lots of shaking
    public static double armKf = 0.000023; // if you want to make it go up or down more basically same as Kp? change both

    // Correction represents the error term of the PIDF loop
    private double correction;
    private double correction2;

    //CHANGE BASED ticks
    public static double HIGH = 1269; // Actual position based on encoder readings
    public static double MIDDLE = 1000; //estimate these both
    public static double LOW = 500; // Actual position based on encoder readings
    public static double GROUND = 0; //estimate these both

    // Initially set to 0 because we only want the claw to move when given input from the controller
    // initializing the targetPos value to a greater positive value would cause the update() method to
    // immediately start moving the arm since a difference between the current motor encoder position
    // and the target position is created (error).
    public double targetPos = 0.0;

    public Slide(HardwareMap hardwareMap){
        LMotor = new Motor(hardwareMap, "ARM", Motor.GoBILDA.RPM_435); // Pin ___ // change the rpm
        RMotor = new Motor(hardwareMap, "ARM", Motor.GoBILDA.RPM_435); // change rpm and maybe name
        LMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        LMotor.setRunMode(Motor.RunMode.VelocityControl);
        RMotor.setRunMode(Motor.RunMode.VelocityControl);

        LMotor.resetEncoder(); // We want the arm to start at a position of 0
        RMotor.resetEncoder();

        double[] PIDF_COEFF = {armKp, armKi, armKd, armKf};
        armPIDF = new PIDFController(PIDF_COEFF[0], PIDF_COEFF[1], PIDF_COEFF[2], PIDF_COEFF[3]);
    }

    public void update(Telemetry telemetry){
        armPIDF.setPIDF(armKp, armKi, armKd, armKf);
        correction = armPIDF.calculate(LMotor.getCurrentPosition(), targetPos);
        correction2 = armPIDF.calculate(RMotor.getCurrentPosition(), targetPos);

        telemetry.addData("Correction L: ", correction);
        telemetry.addData("Target Position L : ", targetPos);
        telemetry.addData("Motor Position L: ", LMotor.getCurrentPosition());

        telemetry.addData("Correction R : ", correction);
        telemetry.addData("Target Position R : ", targetPos);
        telemetry.addData("Motor Position R : ", LMotor.getCurrentPosition());
        telemetry.update();



        LMotor.set(correction);
        RMotor.set(correction2);
    }

    public void setHIGH(){
        targetPos = HIGH;
    }

    public void setMIDDLE(){
        targetPos = MIDDLE;
    }

    public void setLOW(){
        targetPos = LOW;
    }

    public void setGROUND(){
        targetPos = GROUND;
    }
}