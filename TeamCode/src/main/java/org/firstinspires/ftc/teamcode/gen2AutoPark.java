package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="gen2AutoPark")


public class gen2AutoPark extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        DemoBotDriveMecanum drive = new DemoBotDriveMecanum();
        DcMotor[] motors = new DcMotor[4];
        {
            motors[0] = robot.motorLF;
            motors[1] = robot.motorLB;
            motors[2] = robot.motorRF;
            motors[3] = robot.motorRB;

        }

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime whatever = new ElapsedTime();

        waitForStart();

        drive.encoderDrive(1000, driveStyle.FORWARD, 1, motors);

        //drive.timeDrive(100,1,driveStyle.FORWARD,motors);

        //sleep(10000);


    }


}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
