package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name="meet1Auto")


public class meet1Auto extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //ElapsedTime whatever = new ElapsedTime();



        waitForStart();

        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robot.motorLB.setPower(-0.5  * 3/4);
        robot.motorLM.setPower(-0.5);
        robot.motorLF.setPower(-0.5 * 3/4);
        robot.motorRB.setPower(-0.5  * 3/4);
        robot.motorRM.setPower(-0.5);
        robot.motorRF.setPower(-0.5 * 3/4);

        sleep(500);

        robot.motorLB.setPower(0  * 3/4);
        robot.motorLM.setPower(0);
        robot.motorLF.setPower(0 * 3/4);
        robot.motorRB.setPower(0  * 3/4);
        robot.motorRM.setPower(0);
        robot.motorRF.setPower(0 * 3/4);

        while (robot.angles.firstAngle > -90) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.motorLB.setPower(0.5  * 3/4);
            robot.motorLM.setPower(0.5);
            robot.motorLF.setPower(0.5 * 3/4);

            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.update();
        }
        robot.motorLB.setPower(0.5  * 3/4);
        robot.motorLM.setPower(0.5);
        robot.motorLF.setPower(0.5 * 3/4);
        robot.motorRB.setPower(0.5  * 3/4);
        robot.motorRM.setPower(0.5);
        robot.motorRF.setPower(0.5 * 3/4);

        sleep(1500);

        robot.motorLB.setPower(0  * 3/4);
        robot.motorLM.setPower(0);
        robot.motorLF.setPower(0 * 3/4);
        robot.motorRB.setPower(0  * 3/4);
        robot.motorRM.setPower(0);
        robot.motorRF.setPower(0 * 3/4);

        while (robot.angles.firstAngle > -110) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.motorRB.setPower(-0.5  * 3/4);
            robot.motorRM.setPower(-0.5);
            robot.motorRF.setPower(-0.5 * 3/4);

            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.update();
        }

        robot.motorLB.setPower(-0.5  * 3/4);
        robot.motorLM.setPower(-0.5);
        robot.motorLF.setPower(-0.5 * 3/4);
        robot.motorRB.setPower(-0.5  * 3/4);
        robot.motorRM.setPower(-0.5);
        robot.motorRF.setPower(-0.5 * 3/4);

        sleep(700);

        robot.motorLB.setPower(0  * 3/4);
        robot.motorLM.setPower(0);
        robot.motorLF.setPower(0 * 3/4);
        robot.motorRB.setPower(0  * 3/4);
        robot.motorRM.setPower(0);
        robot.motorRF.setPower(0 * 3/4);

        while (robot.angles.firstAngle > -90) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.motorLB.setPower(-0.5  * 3/4);
            robot.motorLM.setPower(-0.5);
            robot.motorLF.setPower(-0.5 * 3/4);

            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.update();
        }

        robot.motorLB.setPower(0  * 3/4);
        robot.motorLM.setPower(0);
        robot.motorLF.setPower(0 * 3/4);
        robot.motorRB.setPower(0  * 3/4);
        robot.motorRM.setPower(0);
        robot.motorRF.setPower(0 * 3/4);

            //robot.motorRF.setPower(gamepad1.right_stick_y * 3/4);
            //robot.motorRM.setPower(gamepad1.right_stick_y);
            //robot.motorRB.setPower(gamepad1.right_stick_y * 3/4);
            //robot.motorLB.setPower(gamepad1.left_stick_y * 3/4);
            //robot.motorLM.setPower(gamepad1.left_stick_y);
            //robot.motorLF.setPower(gamepad1.left_stick_y * 3/4);

    }
}
