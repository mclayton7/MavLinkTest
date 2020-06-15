
namespace App
{
    class Location3d
    {
        public double LatitudeInRadians {get; }
        public double LongitudeInRadians {get; }
        public double AltitudeMslInMeters {get; }

        public Location3d(double latitudeInRadians, double longitudeInRadians, double altitudeMslInMeters)
        {
            LatitudeInRadians = latitudeInRadians;
            LongitudeInRadians = longitudeInRadians;
            AltitudeMslInMeters = altitudeMslInMeters;
        }
    }
}