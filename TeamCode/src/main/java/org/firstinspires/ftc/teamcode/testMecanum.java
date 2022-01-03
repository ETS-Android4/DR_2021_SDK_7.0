package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MecanumTeleOp")
//@Disabled

public class testMecanum extends LinearOpMode
{

    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    public DcMotor intake1 = null;
    public DcMotor intake2 = null;

    double speed = .75;
    double zScale = 1;

    boolean outputSide;
    boolean DpadUpToggle2 = true;
    boolean DpadDownToggle = true;
    int armSetPos;
    double armPos;
    double servoPos;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //ElapsedTime whatever = new ElapsedTime();

        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake1");


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

        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive())
        {


            intake1.setPower(-gamepad1.left_trigger);
            intake2.setPower(-gamepad1.left_trigger);

            intake1.setPower(gamepad1.right_trigger);
            intake2.setPower(gamepad1.right_trigger);

            motorRF.setPower(speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
            motorRB.setPower(speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zScale * gamepad1.right_stick_x)));
            motorLB.setPower(speed*((gamepad1.left_stick_y + gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
            motorLF.setPower(speed*((-gamepad1.left_stick_x + gamepad1.left_stick_y)) - (zScale * gamepad1.right_stick_x));

            if (gamepad2.dpad_down && DpadDownToggle){
                armSetPos = armSetPos - 1;

                DpadDownToggle = false;
            }

            else if (!gamepad2.dpad_down && !DpadDownToggle) {
                DpadDownToggle = true;
            }

            if (gamepad2.dpad_up && DpadUpToggle2){
                armSetPos = armSetPos + 1;

                DpadUpToggle2 = false;
            }

            else if (!gamepad2.dpad_up && !DpadUpToggle2) {
                DpadUpToggle2 = true;
            }

            if (gamepad2.dpad_down || gamepad2.dpad_up)
            {
                if (armSetPos == 1)
                {
                    armPos = 0;
                    servoPos = 1;
                } else if (armSetPos == 2)
                {
                    armPos = 5;
                    servoPos = .33;
                } else if (armSetPos == 3)
                {
                    armPos = 0;
                    servoPos = .16;
                } else if (armSetPos == 4)
                {
                    armPos = 0;
                    servoPos = .05;
                } else if (armSetPos == 5)
                {
                    armPos = 0;
                    servoPos = 0;
                }
            }

            if(gamepad2.dpad_right)
            {
                //set the servos to run to position
                if(outputSide)
                {

                }
                else
                {

                }
            }

            if(gamepad2.dpad_left)
            {
                //return servos to home
            }

            if(gamepad2.right_bumper)
            {
                outputSide = true;
            }
            else if(gamepad2.left_bumper)
            {
                outputSide = false;
            }

            if(outputSide)
            {
                //which side of the robots output is in use
            }

            telemetry.addData("motorRF",motorRF.getPower());
            telemetry.addData("motorRB",motorRB.getPower());
            telemetry.addData("motorLB",motorLB.getPower());
            telemetry.addData("motorLF",motorLF.getPower());

            telemetry.update();

        }
    }
}

