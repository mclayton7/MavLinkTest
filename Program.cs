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

            var udpClient = new UdpClient(14550);
            udpClient.MulticastLoopback = false;
            udpClient.JoinMulticastGroup(IPAddress.Parse("239.255.145.50"));

            var mavparse = new MAVLink.MavlinkParse();
            var mavLink = new MAVLink();
            var mavlinkControl = new MavLinkControl(udpClient);
            var messageTypes = new HashSet<uint>();
            
            var heartbeatTimer = new System.Timers.Timer(1000);
            heartbeatTimer.AutoReset = true;

            heartbeatTimer.Elapsed += async (o, e) =>
            {
                await mavlinkControl.SendHeartbeat();
            };

            Task.Run(async () =>
            {
                heartbeatTimer.Start();
                while (true)
                {
                    var result = await udpClient.ReceiveAsync();
                    var packet = mavparse.ReadPacket(new MemoryStream(result.Buffer));
                    if (packet == null || packet.data == null)
                    {
                        continue;
                    }

                    messageTypes.Add(packet.msgid);

                    switch (packet.msgid)
                    {
                        case (uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT:
                            {
                                var heartbeat = packet.ToStructure<MAVLink.mavlink_heartbeat_t>();

                                if (packet.sysid != 255)
                                {
                                    Console.WriteLine($"got heartbeat, SysId:{packet.sysid} CompId:{packet.compid} Autopilot:{heartbeat.autopilot} Base Mode:{heartbeat.base_mode} System State:{heartbeat.system_status}");
                                }

                                break;
                            }

                        case (uint)MAVLink.MAVLINK_MSG_ID.SYS_STATUS:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.SYSTEM_TIME:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_READ:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.PARAM_VALUE:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.RAW_IMU:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.SCALED_PRESSURE:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT:
                            {
                                var gps = packet.ToStructure<MAVLink.mavlink_global_position_int_t>();
                                double latitudeDegrees = gps.lat / (double)Math.Pow(10, 7);
                                double longitudeDegrees = gps.lon / (double)Math.Pow(10, 7);
                                double altMslMeters = gps.alt / (double)1000;
                                Console.WriteLine($"{latitudeDegrees} {longitudeDegrees} {altMslMeters}");
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.MISSION_CURRENT:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.GPS_GLOBAL_ORIGIN:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.NAV_CONTROLLER_OUTPUT:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.RC_CHANNELS:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.VFR_HUD:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG:
                            {
                                Console.WriteLine("saw command from someone");
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.COMMAND_ACK:
                            {
                                var ack = packet.ToStructure<MAVLink.mavlink_command_ack_t>();
                                Console.WriteLine($"Got an ACK from command. Result: {ack.result}");
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.POSITION_TARGET_GLOBAL_INT:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.FILE_TRANSFER_PROTOCOL:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.TIMESYNC:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.SCALED_IMU2:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.POWER_STATUS:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.SCALED_IMU3:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.TERRAIN_REQUEST:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.TERRAIN_REPORT:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.BATTERY_STATUS:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.SENSOR_OFFSETS:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.MEMINFO:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.AHRS:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.SIMSTATE:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.HWSTATUS:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.WIND:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.AHRS2:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.AHRS3:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.EKF_STATUS_REPORT:
                            {
                                break;
                            }

                        case (uint)MAVLink.MAVLINK_MSG_ID.VIBRATION:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.HOME_POSITION:
                            {
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.STATUSTEXT:
                            {
                                var textPacket = packet.ToStructure<MAVLink.mavlink_statustext_t>();
                                Console.WriteLine($"Status: {System.Text.Encoding.ASCII.GetString(textPacket.text)}");
                                break;
                            }
                        case (uint)MAVLink.MAVLINK_MSG_ID.AOA_SSA:
                            {
                                break;
                            }
                        default:
                            {
                                Console.WriteLine($"Not Handling {packet.msgid}");
                                break;
                            }
                    }
                }
            });

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

                            _ = mavlinkControl.CommandLoiter(latitude, longitude, altitudeMsl);
                            Console.WriteLine("Loiter High");
                            break;
                        }
                    case "loiter-low":
                        {
                            double latitude = -34.3632612 * Math.PI / 180.0;
                            double longitude = 149.1652363 * Math.PI / 180.0;
                            double altitudeMsl = 200.0;

                            _ = mavlinkControl.CommandLoiter(latitude, longitude, altitudeMsl);

                            Console.WriteLine("Loiter Low");
                            break;
                        }
                }
            }
        }
    }
}
