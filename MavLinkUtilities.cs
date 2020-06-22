using System;
using System.Collections.Generic;
using System.Text;

namespace MavLinkTest
{
    public static class MavLinkUtilities
    {
        public static double DegreesToRadians(double degrees)
        {
            return degrees * Math.PI / 180.0;
        }
        public static double RadiansToDegrees(double radians)
        {
            return radians * 180.0 / Math.PI;
        }

        public static int ConvertRadiansToDegreesE7(double radians)
        {
            return (int)(RadiansToDegrees(radians) * 1e7);
        }
    }
}
