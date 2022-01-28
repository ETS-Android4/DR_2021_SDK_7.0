package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="capTest")


public class capTest extends LinearOpMode
{
    public Servo CapVert = null;
    public Servo CapSides = null;
    public CRServo CapOut = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        CapVert = hardwareMap.servo.get("CapVert");
        CapSides = hardwareMap.servo.get("CapSides");
        CapOut = hardwareMap.crservo.get("CapOut");


        ElapsedTime whatever = new ElapsedTime();

        waitForStart();

        while (opModeIsActive())
        {
            CapVert.setPosition(((gamepad2.left_stick_y + 1) * .125));
            CapSides.setPosition((((gamepad2.left_stick_x + 1) * .5) * .35) + .12);
            CapOut.setPower(gamepad2.right_stick_y * .7);
        }
    }


}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
