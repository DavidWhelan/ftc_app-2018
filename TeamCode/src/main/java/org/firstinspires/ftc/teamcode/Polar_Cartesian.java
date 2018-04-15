package org.firstinspires.ftc.teamcode;

/**
 * Created by David on 3/18/2018.
 */

public class Polar_Cartesian
{
    public Polar_Cartesian()
    {

    }

    static public double[] polar(double x, double y)
    {
        double[] coordinate = new double[2];

        coordinate[0] = Math.sqrt(Math.pow(y, 2)+Math.pow(x, 2));
        coordinate[1] = Math.atan2(y, x);

        return coordinate;
    }

    static public double[] cartesian(double radius, double theta)
    {
        double[] coordinate = new double[2];

        coordinate[0] = radius * Math.cos(theta);
        coordinate[1] = radius * Math.sin(theta);

        return coordinate;
    }

    static public double[] polar(double[] cartesian_coordinate)
    {
        if(cartesian_coordinate.length != 2)
        {
            double[] to_return = {0,0};
            return to_return;
        }

        double x = cartesian_coordinate[0];
        double y = cartesian_coordinate[1];

        double[] coordinate = new double[2];

        coordinate[0] = Math.sqrt(Math.pow(y, 2)+Math.pow(x, 2));
        coordinate[1] = Math.atan2(y, x);

        return coordinate;
    }

    static public double[] cartesian(double[] polar_coordinate)
    {
        if(polar_coordinate.length != 2)
        {
            double[] to_return = {0,0};
            return to_return;
        }

        double radius = polar_coordinate[0];
        double theta = polar_coordinate[1];

        double[] coordinate = new double[2];

        coordinate[0] = radius * Math.cos(theta);
        coordinate[1] = radius * Math.sin(theta);

        return coordinate;
    }


}
