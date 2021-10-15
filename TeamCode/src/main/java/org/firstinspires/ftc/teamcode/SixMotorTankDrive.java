package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name="SixMotorTankDrive")


public class SixMotorTankDrive extends LinearOpMode
{

    int armPos;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

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

            armPos = robot.arm.getCurrentPosition();

            if (gamepad2.left_stick_y > -.1 && gamepad2.left_stick_y < .1 && !robot.arm.isBusy())
            {
                    robot.arm.setTargetPosition(armPos);

                    robot.arm.setPower(.5);

                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else
            {
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.arm.setPower(gamepad2.left_stick_y);
            }

                robot.servo3.setPosition(robot.servo3.getPosition() + gamepad2.left_stick_x);

                robot.servo2.setPosition(robot.servo2.getPosition() - gamepad2.right_stick_y);


            if (gamepad2.a) {
                robot.servo.setPosition(0);
            }

            if(gamepad2.b){
                robot.servo.setPosition(0);
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
            telemetry.update();
        }
    }
}
