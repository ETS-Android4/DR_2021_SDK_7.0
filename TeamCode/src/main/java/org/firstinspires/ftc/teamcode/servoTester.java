package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="servoTester")


public class servoTester extends LinearOpMode
{
    //change if using different servos
    public Servo CapVert = null;
    public Servo CapSides = null;
    public CRServo CapOut = null;


    //add as many as you have servos
    double increment = 0;
    double increment2 = 0;
    double increment3 = 0;
    //double increment3 = 0;
    double distance = 0.5;

    boolean kickout;
    int loopCount = 0;


    public void runOpMode() throws InterruptedException
    {

        //change to servos that you are using
        CapVert = hardwareMap.servo.get("CapVert");
        CapSides = hardwareMap.servo.get("CapSides");
        CapOut = hardwareMap.crservo.get("CapOut");


        waitForStart();

        while (opModeIsActive())
        {

            CapVert.setPosition(increment);
            CapSides.setPosition(increment2);
            CapOut.setPower(increment3);

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

            //if the down depad button is pressed then it adds to the first servos position
            if(gamepad1.dpad_down && loopCount > 10000)
            {
                increment3 += distance;
                loopCount = 0;

            }

            //if the right depad button is pressed then it subtract from the first servos position
            if(gamepad1.dpad_right && loopCount > 10000)
            {
                increment3 -= distance;
                loopCount = 0;
            }



            loopCount++;

            telemetry.addData("vert",increment);
            telemetry.addData("sides",increment2);
            telemetry.addData("out",increment3);
            telemetry.addData("vert real",CapVert.getPosition());
            telemetry.addData("sides real",CapSides.getPosition());
            telemetry.addData("out real", CapOut.getPower());
            telemetry.addData("we are changing by :",distance);
            telemetry.addData("loopcount:",loopCount);
            telemetry.update();

        }
    }


}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
