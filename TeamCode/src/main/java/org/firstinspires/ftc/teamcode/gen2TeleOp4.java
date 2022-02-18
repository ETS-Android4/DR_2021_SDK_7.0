package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="gen2TeleOp4")
//@Disabled

public class gen2TeleOp4 extends LinearOpMode
{

    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    public DcMotor intake1 = null;
    public DcMotor intake2 = null;
    public DcMotor duckSpinnerLeft = null;
    public DcMotor duckSpinnerRight = null;
    public Servo baseRight = null;
    public Servo armRight = null;
    public Servo bucketRight = null;
    public Servo slidesL = null;
    public Servo armL = null;
    public Servo clawL = null;
    public Servo CapVert = null;
    public Servo CapSides = null;
    public CRServo CapOut = null;

    double speed = 1;
    double zScale = 1;

    boolean outputSide;
    boolean DpadUpToggle2 = true;
    boolean DpadDownToggle = true;
    boolean MoveUp = false;
    boolean MoveDown = false;
    int armSetPos = 0;
    int armReturnPos = 1;
    double armPos = 2.3;
    double servoPos;
    double ServoTime;
    double ServoTime2;
    boolean blue = false;
    boolean red = false;
    boolean yellow = true;
    double duckPower = 0;
    double CapSidesVar = .3;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime servoTime = new ElapsedTime();

        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        duckSpinnerLeft = hardwareMap.dcMotor.get("duckSpinnerLeft");
        duckSpinnerRight = hardwareMap.dcMotor.get("duckSpinnerRight");
        baseRight = hardwareMap.servo.get("baseRight");
        armRight = hardwareMap.servo.get("armRight");
        bucketRight = hardwareMap.servo.get("bucketRight");
        slidesL = hardwareMap.servo.get("slidesL");
        armL = hardwareMap.servo.get("armL");
        clawL = hardwareMap.servo.get("clawL");
        CapVert = hardwareMap.servo.get("CapVert");
        CapSides = hardwareMap.servo.get("CapSides");
        CapOut = hardwareMap.crservo.get("CapOut");



        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        ServoTime = servoTime.milliseconds();


        while (opModeIsActive())
        {


            if (gamepad1.a)
            {
                duckPower += .015;
                duckSpinnerLeft.setPower(duckPower);
                duckSpinnerRight.setPower(duckPower);
            }
            else if (gamepad1.b)
            {
                duckPower += .015;
                duckSpinnerLeft.setPower(-duckPower);
                duckSpinnerRight.setPower(-duckPower);
            } else
            {
                duckPower = 0;
                duckSpinnerLeft.setPower(0);
                duckSpinnerRight.setPower(0);
            }

            if (gamepad1.left_trigger >= .2)
            {
                intake1.setPower(gamepad1.left_trigger);
                intake2.setPower(gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger >= .2)
            {
                intake1.setPower(-gamepad1.right_trigger * .8);
                intake2.setPower(-gamepad1.right_trigger * .8);
            }
            else
            {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            motorRF.setPower(speed * ((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
            motorRB.setPower(speed * (-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zScale * gamepad1.right_stick_x)));
            motorLB.setPower(speed * ((gamepad1.left_stick_y + gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
            motorLF.setPower(speed * ((-gamepad1.left_stick_x + gamepad1.left_stick_y)) - (zScale * gamepad1.right_stick_x));

            //if(gamepad2.a)
            //{
            //    clawL.setPosition(0);
            //}
            //if(gamepad2.b)
            //{
            //    clawL.setPosition(.15);
            //}
            //if(gamepad2.left_bumper)
            //{
            //    clawL.setPosition(.05);
            //}
            //if(gamepad2.x)
            //{
            //    slidesL.setPosition(0);
            //}
            //if(gamepad2.y)
            //{
            //    slidesL.setPosition(.45);
            //}
            //if(gamepad2.dpad_down)
            //{
            //    slidesL.setPosition(.25);
            //}
            //if(gamepad2.dpad_up)
            //{
            //    armL.setPosition(0);
            //}
            //if(gamepad2.dpad_right)
            //{
            //    armL.setPosition(.75);
            //}
            //if(gamepad2.dpad_left)
            //{
            //    armL.setPosition(.5);
            //}




            if (gamepad2.dpad_down && DpadDownToggle)
            {
                armSetPos = armSetPos - 1;

                DpadDownToggle = false;
            } else if (!gamepad2.dpad_down && !DpadDownToggle)
            {
                DpadDownToggle = true;
            }

            if (gamepad2.dpad_up && DpadUpToggle2)
            {
                armSetPos = armSetPos + 1;

                DpadUpToggle2 = false;
            } else if (!gamepad2.dpad_up && !DpadUpToggle2)
            {
                DpadUpToggle2 = true;
            }






            if (gamepad2.a)
            {
                clawL.setPosition(0);
            }
            if (gamepad2.b)
            {
                clawL.setPosition(.26);

            }
            if (gamepad2.x)
            {
                clawL.setPosition(.15);
            }


            //if (gamepad2.right_bumper)
            //{
            //    outputSide = true;
            //} else if (gamepad2.left_bumper)
            //{
            //    outputSide = false;
            //}

            //if (outputSide)
            //{
            //    //which side of the robots output is in use
            //}

            if (armSetPos == 0)
            {

                if (gamepad2.dpad_right)
                {
                    MoveUp = true;

                }

                if (MoveUp)
                {
                    slidesL.setPosition(0);
                    armL.setPosition(.78);
                    MoveUp = false;
                }


            }


             else if (armSetPos == 1)
            {
                if (gamepad2.dpad_right)
                {
                    MoveUp = true;
                }

                if (MoveUp)
                {
                    slidesL.setPosition(0);
                    armL.setPosition(.65);
                    MoveUp = false;
                }
            }




            else if (armSetPos == 2)
            {
                if (gamepad2.dpad_right)
                {
                    MoveUp = true;
                }

                if (MoveUp)
                {
                    slidesL.setPosition(0);
                    armL.setPosition(.55);
                    MoveUp = false;
                }
            }

            else if (armSetPos == 3)
            {
                if (gamepad2.dpad_right)
                {
                    MoveUp = true;
                }

                if (MoveUp)
                {
                    slidesL.setPosition(0.45);
                    armL.setPosition(.5);
                    MoveUp = false;
                }
            }


            if (gamepad2.dpad_left)
            {
                MoveDown = true;
            }

            if (MoveDown)
            {
                slidesL.setPosition(0);
                armL.setPosition(0);
                clawL.setPosition(.15);
                MoveDown = false;
            }

            if(gamepad2.left_bumper)
            {
                red = true;
                blue = false;
                yellow = false;

                CapSidesVar = .3;
            }
            if(gamepad2.right_bumper)
            {
                red = false;
                blue = true;
                yellow = false;

                CapSidesVar = .12;
            }

            if(gamepad2.y)
            {
                red = false;
                blue = false;
                yellow = true;
            }
            if (yellow)
            {
                CapSides.setPosition(0);
            }
            if(red)
            {

                if (gamepad2.right_stick_x >= .5)
                {
                    CapSidesVar += .001;
                }
                else if (gamepad2.right_stick_x <= -.5)
                {
                    CapSidesVar -= .001;
                }

                CapSides.setPosition(CapSidesVar);

                CapVert.setPosition(((gamepad2.left_stick_y + 1) * .25) + 0.05);
                //CapSides.setPosition((((gamepad2.right_stick_x + 1) * .5) * .35) + .12);
                //CapOut.setPower(gamepad2.right_stick_y * .7);
            }
            if(blue)
            {

                if (gamepad2.right_stick_x >= .5)
                {
                    CapSidesVar += .001;
                }
                else if (gamepad2.right_stick_x <= -.5)
                {
                    CapSidesVar -= .001;
                }

                CapSides.setPosition(CapSidesVar);

                CapVert.setPosition(((gamepad2.left_stick_y + 1) * .25) + 0.05);
                //CapSides.setPosition((gamepad2.right_stick_x + 1) * .12);
                //CapOut.setPower(gamepad2.right_stick_y * .7);
            }

            if (gamepad2.left_trigger >= .2)
            {
                CapOut.setPower(-gamepad2.left_trigger);
            }
            else if (gamepad2.right_trigger >= .2)
            {
                CapOut.setPower(gamepad2.right_trigger);
            }
            else
            {
                CapOut.setPower(0);
            }




            telemetry.addData("motorRFEncoder", motorRF.getCurrentPosition());
            telemetry.addData("motorRBEncoder", motorRB.getCurrentPosition());
            telemetry.addData("motorLBEncoder", motorLB.getCurrentPosition());
            telemetry.addData("motorLFEncoder", motorLF.getCurrentPosition());
            telemetry.addData("motorRF", motorRF.getPower());
            telemetry.addData("motorRB", motorRB.getPower());
            telemetry.addData("motorLB", motorLB.getPower());
            telemetry.addData("motorLF", motorLF.getPower());
            telemetry.addData("armPos", armPos);
            telemetry.addData("!!!armSetPos!!!", armSetPos);
            telemetry.addData("armPos", servoPos);
            telemetry.addData("ServoTimer", ServoTime);
            telemetry.addData("servoTimer", servoTime.milliseconds());

            telemetry.update();

        }
    }
}
