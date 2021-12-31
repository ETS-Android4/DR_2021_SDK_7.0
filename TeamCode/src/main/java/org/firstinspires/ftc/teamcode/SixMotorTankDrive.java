package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;


@TeleOp(name="SixMotorTankDrive")


public class SixMotorTankDrive extends LinearOpMode
{

    int armPos;

    double servo2Pos = 1;

    boolean camera_Active = false;

    boolean rightBumperToggle2 = true;
    boolean leftBumperToggle = true;

    boolean DpadUpToggle2 = true;
    boolean DpadDownToggle = true;

    int armSetPos = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        //robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //ElapsedTime whatever = new ElapsedTime();

        waitForStart();

       // robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive())
        {


           // robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.motorRF.setPower(-gamepad1.right_stick_y * 3/4);
            robot.motorRM.setPower(-gamepad1.right_stick_y);
            robot.motorRB.setPower(-gamepad1.right_stick_y * 3/4);
            robot.motorLB.setPower(-gamepad1.left_stick_y * 3/4);
            robot.motorLM.setPower(-gamepad1.left_stick_y);
            robot.motorLF.setPower(-gamepad1.left_stick_y * 3/4);

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

            if (gamepad2.dpad_down || gamepad2.dpad_up)
            {
                if (armSetPos == 1)
                {
                    armPos = 0;
                    servo2Pos = 1;
                } else if (armSetPos == 2)
                {
                    armPos = 525;
                    servo2Pos = .33;
                } else if (armSetPos == 3)
                {
                    armPos = 700;
                    servo2Pos = .16;
                } else if (armSetPos == 4)
                {
                    armPos = 850;
                    servo2Pos = .05;
                } else if (armSetPos == 5)
                {
                    armPos = 950;
                    servo2Pos = 0;
                }
            }

            if (gamepad2.start) {
                robot.arm.setTargetPosition(armPos);

                robot.arm.setPower(0.75);

                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.servo2.setPosition(servo2Pos);
            }

            //robot.servo2.setPosition(gamepad2.right_stick_y);
            //robot.servo2.setPosition(gamepad2.left_stick_y);

           if (gamepad2.a) {
               robot.servo.setPosition(0);
           }

           if (gamepad2.b){
               robot.servo.setPosition(1);
           }

           if (gamepad2.y){
           robot.INservo1.setPower(.7);
           robot.INservo2.setPower(.7);
           }
            if (!gamepad2.y) {
               robot.INservo1.setPower(0);
               robot.INservo2.setPower(0);
           }

           if (gamepad2.x){
               robot.INservo1.setPower(-.7);
               robot.INservo2.setPower(-.7);
           }
            if(!gamepad2.x){
               robot.INservo1.setPower(0);
               robot.INservo2.setPower(0);
            }

           if(gamepad2.back){

               if (gamepad2.left_bumper && leftBumperToggle)
               {
                   armPos = armPos - 50;

                   leftBumperToggle = false;
               } else if (!gamepad2.left_bumper && !leftBumperToggle)
               {
                   leftBumperToggle = true;
               }

               if (gamepad2.right_bumper && rightBumperToggle2)
               {
                   armPos = armPos + 50;

                   rightBumperToggle2 = false;
               } else if (!gamepad2.right_bumper && !rightBumperToggle2)
               {
                   rightBumperToggle2 = true;
               }
           }







           // telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("servo", robot.servo.getPosition());
            telemetry.addData("servo2", robot.servo2.getPosition());
            telemetry.addData("INservo1", robot.INservo1.getPower());
            telemetry.addData("INservo2", robot.INservo2.getPower());
            telemetry.addData("arm", robot.arm.getPower());
            telemetry.addData("armPos", armPos);
            telemetry.addData("armSetPos", armSetPos);
            //telemetry.addData("wrist pos", gamepad2.left_stick_y);
            telemetry.update();

        }
    }
}
