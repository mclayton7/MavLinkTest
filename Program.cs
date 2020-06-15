using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

namespace App
{
    class Program
    {
        static Task Main(string[] args)
        {
            Console.WriteLine("Starting App");

            var port = 14550;
            var multicastGroup = IPAddress.Parse("239.255.145.50");
            var udpClient = new UdpClient(port);
            udpClient.MulticastLoopback = false;
            udpClient.JoinMulticastGroup(multicastGroup);
            var sendIp = new IPEndPoint(multicastGroup, port);
            var mavlinkService = new UdpMavLinkService(udpClient, sendIp);

            var mavlinkControl = new MavLinkControl(mavlinkService);
            
            Task.Run(() => mavlinkService.ListenToMavlinkPackets());
            
            while (true)
            {
                string line = Console.ReadLine();

                switch(line)
                {
                    case "telemetry":
                        {
                            Console.WriteLine("Requesting datastream");
                            _ = mavlinkControl.RequestDataStream();
                            break;
                        }
                    case "guided":
                        {
                            Console.WriteLine("Setting mode to guided");
                            _ = mavlinkControl.SetMode(MAVLink.COPTER_MODE.GUIDED);
                            break;
                        }
                    case "arm":
                        {
                            _ = mavlinkControl.CommandArm();
                            Console.WriteLine("Arming");
                            break;
                        }
 
                    case "disarm":
                        {
                            _ = mavlinkControl.CommandDisarm();
                            Console.WriteLine("Disarm");
                            break;
                        }
                    case "takeoff":
                        {
                            _ = mavlinkControl.CommandTakeOff();
                            Console.WriteLine("Takeoff");
                            break;
                        }
                    case "land":
                        {
                            _ = mavlinkControl.SetMode(MAVLink.COPTER_MODE.LAND);
                            Console.WriteLine("Land");
                            break;
                        }
                    case "loiter-high":
                        {
                            double latitude = -35.3632612 * Math.PI / 180.0;
                            double longitude = 149.1652363 * Math.PI / 180.0;
                            double altitudeMsl = 624.02;

                            _ = mavlinkControl.CommandLoiter(new Location3d(latitude, longitude, altitudeMsl));
                            Console.WriteLine("Loiter High");
                            break;
                        }
                    case "loiter-low":
                        {
                            double latitude = -34.3632612 * Math.PI / 180.0;
                            double longitude = 149.1652363 * Math.PI / 180.0;
                            double altitudeMsl = 200.0;

                            _ = mavlinkControl.CommandLoiter(new Location3d(latitude, longitude, altitudeMsl));

                            Console.WriteLine("Loiter Low");
                            break;
                        }
                }
            }
        }
    }
}
