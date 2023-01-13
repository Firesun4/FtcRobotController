//package org.firstinspires.ftc.teamcode.MechanismTemplates;

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slide {

    private PIDFController slidePIDF;
    private Motor slideLeft, slideRight;

    //TODO: change values into actual tested positions instead of placeholders
    public static double HIGH_JUNCTION= 2200; //<-- 12.90V, 2150 ;
    public static double MID_JUNCTION = 1700; //<-- 12.90V, 1550 ;
    public static double LOW_JUNCTION = 1400;  //<-- 12.90V, 500 ;
    public static double ZERO_POSITION = 100;//5V;
    private final double MAX = 2500;

    private static double slideKp = 0.006; //0.00326; //0.0039;
    private static double slideKi = 0.000001; //0.00000325;
    private static double slideKd = 0.0000012; //0.000001;
    private static double slideKf = 0.000013; //0.000069;

    private final double[] PIDF_COFFECIENTS = {slideKp, slideKi, slideKd, slideKf};

    private double targetPos;
    double correctionLeft;
    double correctionRight;

    public Slide(HardwareMap hardwareMap){
        slideLeft = new Motor(hardwareMap, "SA", Motor.GoBILDA.RPM_435); // Pin 0 on expansion hub
        slideRight = new Motor(hardwareMap,"SB", Motor.GoBILDA.RPM_435); // Pin 1 on control hub

        slideLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideLeft.setRunMode(Motor.RunMode.VelocityControl);
        slideRight.setRunMode(Motor.RunMode.VelocityControl);

        slideLeft.setInverted(true);

        slidePIDF = new PIDFController(PIDF_COFFECIENTS[0],PIDF_COFFECIENTS[1],PIDF_COFFECIENTS[2],PIDF_COFFECIENTS[3]);

        targetPos = ZERO_POSITION; // target position is 0 by default
        slideRight.resetEncoder();
        slideLeft.resetEncoder();
    }

    public void update(Telemetry telemetry){
        //slidePIDF.setPIDF(slideKp, slideKi, slideKd, slideKf);

        correctionLeft = slidePIDF.calculate(slideLeft.getCurrentPosition(), targetPos);
        correctionRight = slidePIDF.calculate(slideRight.getCurrentPosition(), targetPos);


       telemetry.addData("targetPosition: ", targetPos);
        telemetry.addData("Right motor position: ", slideRight.getCurrentPosition());
        telemetry.addData("Left motor position: ", slideLeft.getCurrentPosition());
        telemetry.addData("Left correction: ", correctionLeft);
        telemetry.addData("Right correction: ", correctionRight);
        telemetry.update();




        // sets the output power of the motor
        slideLeft.set(correctionLeft);
        slideRight.set(correctionRight);
    }

    public void manualSlides(int slideIncrement){
        if((targetPos + slideIncrement) <= 100 && (targetPos - slideIncrement) >= 0) {
            targetPos += slideIncrement;
        }
    }

    public void setCustom(double x){targetPos = x;}
    public void setGROUND(){
        targetPos = ZERO_POSITION;
    }
    public void setAboveG(){
        targetPos = 300;
    }

    public void setLOW(){
        targetPos = LOW_JUNCTION;
    }

    public void setMIDDLE(){
        targetPos = MID_JUNCTION;
    }

    public void setHIGH(){
        targetPos = HIGH_JUNCTION;


    }

    public void setManualSlide(int increment){
        double targetManual = (slideRight.getCurrentPosition() + slideLeft.getCurrentPosition())/2 + increment;
        if(targetManual <= MAX && targetManual>= ZERO_POSITION)
            targetPos = targetManual;

    }

}
/*
package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide {
    private PIDFController armPIDF;
    private Motor LMotor;
    private Motor RMotor;
    //private DcMotorEx LMotor, RMotor;

    // Declaring and Initializing PIDF values

    //CHANGE THESE
    public static double armKp = 0.0001; // change first and start at 0   old is 0.00097
    public static double armKi = 0.000001; // if you always have a equal amount of error make higher
    public static double armKd = 0; // only used if lots of shaking
    public static double armKf = 0.000023; // if you want to make it go up or down more basically same as Kp? change both

    // Correction represents the error term of the PIDF loop
    private double correction;
    private double correction2;

    //CHANGE BASED ticks
    public static double HIGH = 50; // Actual position based on encoder readings
    public static double MIDDLE = 20; //estimate these both
    public static double LOW = 10; // Actual position based on encoder readings
    public static double GROUND = 0; //estimate these both

    // Initially set to 0 because we only want the claw to move when given input from the controller
    // initializing the targetPos value to a greater positive value would cause the update() method to
    // immediately start moving the arm since a difference between the current motor encoder position
    // and the target position is created (error).
    public double targetPos = 0.0;

    public Slide(HardwareMap hardwareMap){
        //LMotor = (DcMotorEx) hardwareMap.dcMotor.get("SA");
        //RMotor = (DcMotorEx) hardwareMap.dcMotor.get("SB");
        LMotor = new Motor(hardwareMap, "SA", Motor.GoBILDA.RPM_435); // Pin ___ // change the rpm
        RMotor = new Motor(hardwareMap, "SB", Motor.GoBILDA.RPM_435); // change rpm and maybe name
        LMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        LMotor.setRunMode(Motor.RunMode.VelocityControl);
        RMotor.setRunMode(Motor.RunMode.VelocityControl);
        //LMotor.setTargetPosition((int)targetPos);
        //RMotor.setTargetPosition((int)targetPos);

        LMotor.resetEncoder(); // We want the arm to start at a position of 0
        RMotor.resetEncoder();

        double[] PIDF_COEFF = {armKp, armKi, armKd, armKf};
        armPIDF = new PIDFController(PIDF_COEFF[0], PIDF_COEFF[1], PIDF_COEFF[2], PIDF_COEFF[3]);
    }

    public void update(Telemetry telemetry){
        targetPos = HIGH;
        armPIDF.setPIDF(armKp, armKi, armKd, armKf);
       // LMotor.setTargetPosition((int)targetPos);
       // RMotor.setTargetPosition((int)targetPos);
        correction = armPIDF.calculate(LMotor.getCurrentPosition(), targetPos);
        correction2 = armPIDF.calculate(RMotor.getCurrentPosition(), targetPos);

        telemetry.addData("Correction L: ", correction);
        telemetry.addData("Target Position L : ", targetPos);
        telemetry.addData("Motor Position L: ", LMotor.getCurrentPosition());

        telemetry.addData("Correction R : ", correction2);
        telemetry.addData("Target Position R : ", targetPos);
        telemetry.addData("Motor Position R : ", RMotor.getCurrentPosition());
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
*/
