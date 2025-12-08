package pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()

         .useSecondaryDrivePIDF(true)

        .mass(4)
        .forwardZeroPowerAcceleration(-26.99)
        .lateralZeroPowerAcceleration(-50.48)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.053, 0, 0.003, 0.02))
        .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.01, 0.02))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(1,0.0,0.015,0.6,0.019))
        .centripetalScaling(0.0006);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5
            , 2.5);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                /* other builder steps */
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")

            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .xVelocity(82.28)
            .yVelocity(74.57);

            //.rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)

    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .angularScalar(0.99)


    .linearScalar(1);



}
