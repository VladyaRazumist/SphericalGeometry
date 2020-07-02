using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace SphericalGeometry
{
    class SphericalGeometry
    {
        private const double R = 6371;

        public static double CalculatePolygonArea(IReadOnlyCollection<GeoCoordinate> coordinates)
        {
            var coords = new List<GeoCoordinate>();
            coords.AddRange(coordinates);
            coords.Add(coords[0]);
            const double R = 6371e3;

            var nVertices = coords.Count - 1;
            var S = 0.0; // spherical excess in steradians

            for (var v = 0; v < nVertices; v++)
            {
                var φ1 = DegreesToRadians(coords[v].Latitude);
                var φ2 = DegreesToRadians(coords[v + 1].Latitude);
                var Δλ = DegreesToRadians(coords[v + 1].Longitude - coords[v].Longitude);
                var E = 2 * Math.Atan2(Math.Tan(Δλ / 2) * (Math.Tan(φ1 / 2) + Math.Tan(φ2 / 2)),
                            1 + Math.Tan(φ1 / 2) * Math.Tan(φ2 / 2));
                S += E;
            }

            if (IsPoleEnclosedBy(coords))
                S = Math.Abs(S) - 2 * Math.PI;

            var A = Math.Abs(S * R * R); // area in units of R

            return A;
        }

        private static bool IsPoleEnclosedBy(IReadOnlyCollection<GeoCoordinate> coords)
        {
            var p = coords.ToArray();
            var ΣΔ = 0.0;
            var initBrng = 0.0;
            var prevBrng = InitialBearingBetweenPoints(p[0], p[1]);

            for (var v = 0; v < p.Length - 1; v++)
            {
                initBrng = InitialBearingBetweenPoints(p[v], p[v + 1]);
                var finalBrng = FinalBearingBetweenPoints(p[v], p[v + 1]);
                ΣΔ += (initBrng - prevBrng + 540) % 360 - 180;
                ΣΔ += (finalBrng - initBrng + 540) % 360 - 180;
                prevBrng = finalBrng;
            }

            initBrng = InitialBearingBetweenPoints(p[0], p[1]);
            ΣΔ += (initBrng - prevBrng + 540) % 360 - 180;
            var enclosed = Math.Abs(ΣΔ) < 90; // 0°-ish
            return enclosed;
        }


        public static double DistanceToLineSegment(GeoCoordinate pathStart, GeoCoordinate pathEnd, GeoCoordinate point)
        {
            // We need to check if perpendicular falls on path, it is performed only
            // if two of angles in triangle is sharp. If one of the angles > 90
            // we take as our distance min distance between our point and path ends.

            var crossTrackDistance = CrossTrackDistance(pathStart, pathEnd, point);

            if (!IsAnglesSharp(pathStart, pathEnd, point))
            {
                var d1 = DistanceBetweenPoints(pathStart, point);
                var d2 = DistanceBetweenPoints(pathEnd, point);

                return Math.Min(d1, d2) * 1000;
            }

            return crossTrackDistance;
        }

        public static double CrossTrackDistance(GeoCoordinate pathStart, GeoCoordinate pathEnd, GeoCoordinate point)
        {
            var δ13 = DistanceBetweenPoints(pathStart, point) / R;
            var θ13 = DegreesToRadians(InitialBearingBetweenPoints(pathStart, point));
            var θ12 = DegreesToRadians(InitialBearingBetweenPoints(pathStart, pathEnd));
            var δxt = Math.Asin(Math.Sin(δ13) * Math.Sin(θ13 - θ12));

            return Math.Abs(δxt * R) * 1000;
        }

        public static double DistanceBetweenPoints(GeoCoordinate point1, GeoCoordinate point2)
        {
            // a = sin²(Δφ/2) + cos(φ1)⋅cos(φ2)⋅sin²(Δλ/2)
            // δ = 2·atan2(√(a), √(1−a))
            // see mathforum.org/library/drmath/view/51879.html for derivation

            var φ1 = DegreesToRadians(point1.Latitude);
            var λ1 = DegreesToRadians(point1.Longitude);
            var φ2 = DegreesToRadians(point2.Latitude);
            var λ2 = DegreesToRadians(point2.Longitude);

            var Δφ = φ2 - φ1;
            var Δλ = λ2 - λ1;

            var a = Math.Sin(Δφ / 2) * Math.Sin(Δφ / 2) +
                    Math.Cos(φ1) * Math.Cos(φ2) * Math.Sin(Δλ / 2) * Math.Sin(Δλ / 2);
            var c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
            var d = R * c;

            return d;
        }
        public static double InitialBearingBetweenPoints(GeoCoordinate point1, GeoCoordinate point2)
        {
            // tanθ = sinΔλ⋅cosφ2 / cosφ1⋅sinφ2 − sinφ1⋅cosφ2⋅cosΔλ
            // see mathforum.org/library/drmath/view/55417.html for derivation

            var φ1 = DegreesToRadians(point1.Latitude);
            var φ2 = DegreesToRadians(point2.Latitude);

            var Δλ = DegreesToRadians(point2.Longitude - point1.Longitude);

            var x = Math.Cos(φ1) * Math.Sin(φ2) - Math.Sin(φ1) * Math.Cos(φ2) * Math.Cos(Δλ);
            var y = Math.Sin(Δλ) * Math.Cos(φ2);
            var θ = Math.Atan2(y, x);

            var bearing = RadiansToDegrees(θ);

            return Wrap360(bearing);
        }

        public static double FinalBearingBetweenPoints(GeoCoordinate point1, GeoCoordinate point2)
        {
            var bearing = InitialBearingBetweenPoints(point1, point2) + 180;
            return Wrap360(bearing);
        }

        public static bool IsAnglesSharp(GeoCoordinate pathStart, GeoCoordinate pathEnd, GeoCoordinate point)
        {
            var p1 = ConvertSphericalToCartesian(pathStart);
            var p2 = ConvertSphericalToCartesian(pathEnd);
            var p3 = ConvertSphericalToCartesian(point);

            var vector1 = new Vector3((float)(p2.X - p1.X), (float)(p2.Y - p1.Y), (float)(p2.Z - p1.Z));
            var vector2 = new Vector3((float)(p3.X - p1.X), (float)(p3.Y - p1.Y), (float)(p3.Z - p1.Z));

            var vector3 = new Vector3((float)(p1.X - p2.X), (float)(p1.Y - p2.Y), (float)(p1.Z - p2.Z));
            var vector4 = new Vector3((float)(p3.X - p2.X), (float)(p3.Y - p2.Y), (float)(p3.Z - p2.Z));

            var product1 = Vector3.Dot(vector1, vector2);
            var product2 = Vector3.Dot(vector3, vector4);

            return !(product1 < 0) && !(product2 < 0);
        }

        public static Cartesian ConvertSphericalToCartesian(GeoCoordinate geoPoint)
        {
            var lat = DegreesToRadians(geoPoint.Latitude);
            var lon = DegreesToRadians(geoPoint.Longitude);
            var x = R * Math.Cos(lat) * Math.Cos(lon);
            var y = R * Math.Cos(lat) * Math.Sin(lon);
            var z = R * Math.Sin(lat);
            return new Cartesian(x, y, z);
        }

        public static double Wrap360(double degrees)
        {
            if (0 <= degrees && degrees < 360) return degrees; // avoid rounding due to arithmetic ops if within range
            return (degrees % 360 + 360) % 360; // sawtooth wave p:360, a:360
        }

        public static double RadiansToDegrees(double radians)
        {
            return radians * (180 / Math.PI);
        }

        public static double DegreesToRadians(double degrees)
        {
            return degrees * (Math.PI / 180);
        }

        public class GeoCoordinate
        {
            public double Latitude { get; }
            public double Longitude { get; }

            public GeoCoordinate(double lat, double lon)
            {
                Latitude = lat;
                Longitude = lon;
            }
        }

        public class Cartesian
        {
            public double X { get; }
            public double Y { get; }
            public double Z { get; }

            public Cartesian(double x, double y, double z)
            {
                X = x;
                Y = y;
                Z = z;
            }
        }
    }
}
