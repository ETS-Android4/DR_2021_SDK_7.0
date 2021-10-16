package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="SixMotorTankDrive")


public class SixMotorTankDrive extends LinearOpMode
{

    int armPos;

    double servo2Pos = 1;

    boolean rightBumperToggle2 = true;
    boolean leftBumperToggle = true;

    boolean DpadUpToggle2 = true;
    boolean DpadDownToggle = true;

    int armSetPos = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);;


        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //ElapsedTime whatever = new ElapsedTime();

        waitForStart();

       // robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive())
        {
           // robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.motorRF.setPower(gamepad1.right_stick_y * 3/4);
            robot.motorRM.setPower(gamepad1.right_stick_y);
            robot.motorRB.setPower(gamepad1.right_stick_y * 3/4);
            robot.motorLB.setPower(gamepad1.left_stick_y * 3/4);
            robot.motorLM.setPower(gamepad1.left_stick_y);
            robot.motorLF.setPower(gamepad1.left_stick_y * 3/4);

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

            //set correct positions, motor spazes out. solution unkown

            if (armSetPos == 1){
                armPos = 0;
                robot.servo2.setPosition(1);
            }

            else if (armSetPos == 2){
                armPos = 400;
                robot.servo2.setPosition(.7);
            }

            else if (armSetPos == 3){
                armPos = 700;
                robot.servo2.setPosition(.4);
            }

            else if (armSetPos == 4){
                armPos = 1000;
                robot.servo2.setPosition(0);
            }

            if (gamepad2.left_bumper && leftBumperToggle){
                armPos = armPos - 50;

                leftBumperToggle = false;
            }

            else if (!gamepad2.left_bumper && !leftBumperToggle) {
                leftBumperToggle = true;
            }

            if (gamepad2.right_bumper && rightBumperToggle2){
                armPos = armPos + 50;

                rightBumperToggle2 = false;
            }

            else if (!gamepad2.right_bumper && !rightBumperToggle2) {
                rightBumperToggle2 = true;
            }

                    robot.arm.setTargetPosition(armPos);

                    robot.arm.setPower(1);

                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                robot.servo3.setPosition(((gamepad2.left_stick_x + 1)/2 ) * .65);

                //robot.servo2.setPosition(gamepad2.right_stick_y);



            robot.servo2.setPosition(servo2Pos);


           if (gamepad2.a) {
               robot.servo.setPosition(0);
           }

           if (gamepad2.b){
               robot.servo.setPosition(1);
           }

           if (gamepad2.x){
           robot.INservo1.setPower(.7);
           robot.INservo2.setPower(.3);
           }
           else {
               robot.INservo1.setPower(0);
               robot.INservo2.setPower(0);
           }

           if (gamepad2.y){
               robot.INservo1.setPower(.3);
               robot.INservo2.setPower(.7);
           }
           else {
               robot.INservo1.setPower(0);
               robot.INservo2.setPower(0);
            }






           // telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("servo", robot.servo.getPosition());
            telemetry.addData("servo2", robot.servo2.getPosition());
            telemetry.addData("servo3", robot.servo3.getPosition());
            telemetry.addData("INservo1", robot.INservo1.getPower());
            telemetry.addData("arm", robot.arm.getPower());
            telemetry.addData("armPos", robot.arm.getCurrentPosition());
            telemetry.addData("armSetPos", armSetPos);
            telemetry.update();
        }
    }
}
