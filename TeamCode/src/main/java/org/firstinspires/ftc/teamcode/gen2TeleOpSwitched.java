package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="gen2TeleOpSwitched")
//@Disabled

public class gen2TeleOpSwitched extends LinearOpMode
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
    int armSetPos = 2;
    int armReturnPos = 1;
    double armPos = 2.3;
    double servoPos;
    double ServoTime;
    double ServoTime2;



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
                duckSpinnerLeft.setPower(1);
                duckSpinnerRight.setPower(1);
            }
            else if (gamepad1.b)
            {
                duckSpinnerLeft.setPower(-1);
                duckSpinnerRight.setPower(-1);
            } else
            {
                duckSpinnerLeft.setPower(0);
                duckSpinnerRight.setPower(0);
            }

            if (gamepad1.left_trigger >= .2)
            {
                intake1.setPower(gamepad1.left_trigger * .5);
                intake2.setPower(gamepad1.left_trigger * .5);
            }
            else if (gamepad1.right_trigger >= .2)
            {
                intake1.setPower(-gamepad1.right_trigger * .5);
                intake2.setPower(-gamepad1.right_trigger * .5);
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

            /* if (gamepad2.dpad_down || gamepad2.dpad_up)
            {
                if (armSetPos == 1)
                {
                    armPos = .23;

                }

                else if (armSetPos == 2)
                {
                    armPos = .4;

                }
                else if (armSetPos == 3)
                {
                    armPos = .6;

                }
            } */

            if (gamepad2.a)
            {
                armRight.setPosition(0);
            }
            if (gamepad2.b)
            {
                armRight.setPosition(.1);

            }
            if (gamepad2.x)
            {
                armRight.setPosition(.55);
            }


            //if(gamepad2.dpad_right)
            //{
            //    //set the servos to run to position
            //    baseRight.setPosition(armPos);
//
            //}
//
            //if(gamepad2.dpad_left)
            //{
            //    armSetPos = 1;
            //    //return servos to home
            //}

            if (gamepad2.right_bumper)
            {
                outputSide = true;
            } else if (gamepad2.left_bumper)
            {
                outputSide = false;
            }

            if (outputSide)
            {
                //which side of the robots output is in use
            }

            if (armSetPos == 2)
            {

                if (gamepad2.dpad_right)
                {
                    MoveUp = true;
                    servoTime.reset();
                }

                if (MoveUp)
                {
                    bucketRight.setPosition(.17);

                    armRight.setPosition(.1);


                    if (servoTime.milliseconds() >= 500)
                    {
                        baseRight.setPosition(.6);

                        MoveUp = false;

                        //armReturnPos = 2;
                    }
                }

            } else if (armSetPos == 1)
            {
                if (gamepad2.dpad_right)
                {
                    MoveUp = true;
                    servoTime.reset();
                }

                if (MoveUp)
                {
                    bucketRight.setPosition(.17);

                    armRight.setPosition(.1);


                    if (servoTime.milliseconds() >= 500)
                    {
                        baseRight.setPosition(.6);

                        //MoveUp = false;

                        if (servoTime.milliseconds() >= 1000)
                        {
                            bucketRight.setPosition(0);

                            armRight.setPosition(.3);

                            if (servoTime.milliseconds() >= 1200)
                            {
                                baseRight.setPosition(.23);

                                MoveUp = false;

                                //armReturnPos = 1;
                            }
                        }
                    }
                }
            }

            //else if (armSetPos == 0)
            //{
            //    if (gamepad2.dpad_right)
            //    {
            //        MoveUp = true;
            //        servoTime.reset();
            //    }
//
            //    if (MoveUp)
            //    {
            //        bucketRight.setPosition(.17);
//
            //        armRight.setPosition(.1);
//
//
            //        if (servoTime.milliseconds() >= 500)
            //        {
            //            baseRight.setPosition(.6);
//
            //            //MoveUp = false;
//
            //            if (servoTime.milliseconds() >= 1000)
            //            {
            //                bucketRight.setPosition(0);
//
            //                armRight.setPosition(.3);
//
            //                if (servoTime.milliseconds() >= 1200)
            //                {
            //                    baseRight.setPosition(.23);
//
            //                    MoveUp = false;
//
            //                    //armReturnPos = 0;
            //                }
            //            }
            //        }
            //    }
            //}


            if (gamepad2.dpad_left)
            {
                MoveDown = true;
                servoTime.reset();
            }

            if (MoveDown)
            {
                baseRight.setPosition(.6);


                if (servoTime.milliseconds() >= 500)
                {
                    bucketRight.setPosition(.17);

                    armRight.setPosition(.1);

                    if (servoTime.milliseconds() >= 1000)
                    {
                        baseRight.setPosition(.23);

                        if (servoTime.milliseconds() >= 2000)
                        {
                            armRight.setPosition(0);

                            MoveDown = false;
                        }
                    }


                }

            }

            CapVert.setPosition(-gamepad2.left_stick_y * .05);
            CapSides.setPosition(gamepad2.left_stick_x * .05);
            CapOut.setPower(-gamepad2.right_stick_y * .05);





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


