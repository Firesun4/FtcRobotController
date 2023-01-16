//package org.firstinspires.ftc.teamcode.MechanismTemplates;
/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.SignalEdgeDetector;

import java.util.function.BooleanSupplier;

@Config
public class Claw {
    public Servo wristJoint;
 //   public Servo clawJoint; // public because a button must set the wrist to the intake position regardless of its status
    private boolean isAuto;

    // claw positions
    public static double OPEN = 0.75;
    public static double CLOSE = 0.35;
    private double pos;

    // wrist positions
    public static double WRIST_INTAKE_POSITION = 0.25; // wrist rotates to intake cone, greater values move clockwise, less move counterclockwise
    public static double WRIST_EXTAKE_POSITION = 0.515; // wrist rotates to extake on junction

  //  SignalEdgeDetector isOpen;
   // SignalEdgeDetector isIntakePosition;

    public Claw(HardwareMap hardwareMap, BooleanSupplier rightBumper, BooleanSupplier aButton) {

        // Control Hub Pins
        wristJoint = hardwareMap.get(Servo.class, "CC"); // Pin 0
        //clawJoint = hardwareMap.get(Servo.class, "CLAW"); // Pin 1

        // Presets for teleOp
     //   clawJoint.setPosition(OPEN);
    //    wristJoint.setPosition(WRIST_INTAKE_POSITION);

    //    isOpen = new SignalEdgeDetector(rightBumper);
     //   isIntakePosition = new SignalEdgeDetector(aButton);
    }

    // Overloaded method for autonomous
    public Claw(HardwareMap hardwareMap){
        // Control Hub Pins
        wristJoint = hardwareMap.get(Servo.class, "CC"); // Pin 0
   //     wristJoint.setPosition(WRIST_INTAKE_POSITION);
    }

    private boolean clawToggled;


    public boolean wristInExtakePosition;

    public void toggleWristRotate() { // would be used at the beginning of the goToJunction method in the arm class, for example
        if (pos == WRIST_EXTAKE_POSITION) {
            wristJoint.setPosition(WRIST_INTAKE_POSITION);
            pos = WRIST_INTAKE_POSITION;
        } else { // if the wrist is in the extake position, switch it back to the intake position so it can pick up cones
            wristJoint.setPosition(WRIST_EXTAKE_POSITION);
            pos = WRIST_EXTAKE_POSITION;
        }
        wristInExtakePosition = !wristInExtakePosition;
    }

}

 */