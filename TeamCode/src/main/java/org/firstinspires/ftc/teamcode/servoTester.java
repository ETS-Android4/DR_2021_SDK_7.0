package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="servoTester")


public class servoTester extends LinearOpMode
{
    //change if using different servos
    public Servo CapVert = null;
    public Servo CapSides = null;


    //add as many as you have servos
    double increment = 0;
    double increment2 = 0;
    //double increment3 = 0;
    double distance = 0.5;

    boolean kickout;
    int loopCount = 0;


    public void runOpMode() throws InterruptedException
    {

        //change to servos that you are using
        CapVert = hardwareMap.servo.get("CapVert");
        CapSides = hardwareMap.servo.get("CapSides");


        waitForStart();

        while (opModeIsActive())
        {

            CapVert.setPosition(increment);
            CapSides.setPosition(increment2);

            if(gamepad1.back)
            {
                distance = .1;
            }
            else
            {
                distance = .05;
            }

            //if the A button is pressed then it adds to the first servos position
            if(gamepad1.a && loopCount > 10000)
            {
                increment += distance;
                loopCount = 0;

            }

            //if the B button is pressed then it subtract from the first servos position
            if(gamepad1.b && loopCount > 10000)
            {
                increment -= distance;
                loopCount = 0;
            }

            //add to servo 2
            if(gamepad1.x && loopCount > 10000)
            {
                increment2 += distance;
                loopCount = 0;
            }

            //subtract from servo 2
            if(gamepad1.y && loopCount > 10000)
            {
                increment2 -= distance;
                loopCount = 0;
            }



            loopCount++;

            telemetry.addData("up and down in code",increment);
            telemetry.addData("up and down real",CapVert.getPosition());
            telemetry.addData("left and right in code",increment2);
            telemetry.addData("up and down real",CapSides.getPosition());
            telemetry.addData("we are changig by :",distance);
            telemetry.addData("loopcount:",loopCount);
            telemetry.update();

        }
    }


}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
