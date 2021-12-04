package org.firstinspires.ftc.teamcode.AutoPID;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous
public class OpenLoopDrivetrainExample extends LinearOpMode {

    // we only have two motors for the sake of keeping the example simple but you are free to add as many as you need!
    public DcMotor motorRF;
    public DcMotor motorRB;
    public DcMotor motorRM;
    public DcMotor motorLM;
    public DcMotor motorLB;
    public DcMotor motorLF;

    @Override
    public void runOpMode() throws InterruptedException {
        // configure your motors and other hardware stuff here
        // make sure the strings match the names that you have set on your robot controller configuration
        motorRF = hardwareMap.get(DcMotor.class,"motorRF");
        motorRB = hardwareMap.get(DcMotor.class,"motorRB");
        motorRM = hardwareMap.get(DcMotor.class,"motorRM");
        motorLM = hardwareMap.get(DcMotor.class,"motorLM");
        motorLB = hardwareMap.get(DcMotor.class,"motorLB");
        motorLF = hardwareMap.get(DcMotor.class,"motorLF");

        // reverse the appropriate motor
        motorLM.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);

        // wait until the start button is pressed
        waitForStart();

        // start motors at 100% power
        motorRF.setPower(1);
        motorRB.setPower(1);
        motorRM.setPower(1);
        motorLM.setPower(1);
        motorLF.setPower(1);
        motorLB.setPower(1);

        // leave motors running for one second
        sleep(1000);

        // stop motors after one second
        motorRF.setPower(0);
        motorRB.setPower(0);
        motorRM.setPower(0);
        motorLM.setPower(0);
        motorLF.setPower(0);
        motorLB.setPower(0);

    }

}
