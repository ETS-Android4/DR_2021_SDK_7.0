package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name="meet1AutoEx")


public class meet1AutoEx extends LinearOpMode
{

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //ElapsedTime whatever = new ElapsedTime();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);





        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

       while (angles.firstAngle > -90) {
           angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

           robot.motorLB.setPower(0.5  * 3/4);
           robot.motorLM.setPower(0.5);
           robot.motorLF.setPower(0.5 * 3/4);

           telemetry.addData("heading", angles.firstAngle);
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

       while (angles.firstAngle > -110) {
           angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

           robot.motorRB.setPower(-0.5  * 3/4);
           robot.motorRM.setPower(-0.5);
           robot.motorRF.setPower(-0.5 * 3/4);

           telemetry.addData("heading", angles.firstAngle);
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

        while (angles.firstAngle > -90) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.motorLB.setPower(-0.5  * 3/4);
            robot.motorLM.setPower(-0.5);
            robot.motorLF.setPower(-0.5 * 3/4);

            telemetry.addData("heading", angles.firstAngle);
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
