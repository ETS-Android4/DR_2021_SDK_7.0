package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="servoTester")


public class servoTester extends LinearOpMode
{
    //change if using different servos
    public Servo slidesL = null;
    public Servo armL = null;
    public Servo clawL = null;


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
        armL = hardwareMap.servo.get("armL");
        slidesL = hardwareMap.servo.get("slidesL");
        clawL = hardwareMap.servo.get("clawL");


        waitForStart();

        while (opModeIsActive())
        {

            slidesL.setPosition(increment);
            armL.setPosition(increment2);
            clawL.setPosition(increment3);

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

            telemetry.addData("slidesL",increment);
            telemetry.addData("armL",increment2);
            telemetry.addData("clawL",increment3);
            telemetry.addData("slidesL real",slidesL.getPosition());
            telemetry.addData("armL real",armL.getPosition());
            telemetry.addData("clawL real", clawL.getPosition());
            telemetry.addData("we are changing by :",distance);
            telemetry.addData("loopcount:",loopCount);
            telemetry.update();

        }
    }


}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
