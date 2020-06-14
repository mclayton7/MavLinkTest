using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;

namespace App
{
    class MavLinkControl
    {
        private readonly UdpClient _udpClient;
        private readonly IPEndPoint _sendIp;

        private readonly MAVLink.MavlinkParse _mavParse = new MAVLink.MavlinkParse();
        private int _sequenceNumber = 0;
        HashSet<uint> _messageTypes = new HashSet<uint>();
        System.Timers.Timer _heartbeatTimer = new System.Timers.Timer(1000);

        public byte GcsSystemId { get; } = 255;
        public byte GcsComponentId { get; } = 190;

        public byte AutopilotSystemId { get; } = 1;
        public byte AutopilotComponentId { get; } = 1;

        public MavLinkControl(UdpClient client, IPEndPoint sendIP)
        {
            _udpClient = client;
            _sendIp = sendIP;

            _heartbeatTimer.AutoReset = true;
            _heartbeatTimer.Elapsed += async (o, e) =>
            {
                await SendHeartbeat();
            };

            _heartbeatTimer.Start();
        }

        public async Task SendHeartbeat()
        {
            var sendpacket = _mavParse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT,
            new MAVLink.mavlink_heartbeat_t()
            {
                autopilot = (byte)MAVLink.MAV_AUTOPILOT.INVALID,
                base_mode = 0,
                custom_mode = 0,
                mavlink_version = 3,
                system_status = 0,
                type = (byte)MAVLink.MAV_TYPE.GCS
            }, false, GcsSystemId, GcsComponentId, _sequenceNumber++);

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, _sendIp);
        }

        public async Task CommandTakeOff()
        {
            var sendpacket = _mavParse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,
            new MAVLink.mavlink_command_long_t()
            {
                target_system = AutopilotSystemId,
                target_component = AutopilotComponentId,
                command = (ushort)MAVLink.MAV_CMD.TAKEOFF,
                param1 = 0,
                param2 = 0,
                param3 = 0,
                param4 = 0,
                param5 = 0,
                param6 = 0,
                param7 = 40,
            }, false, GcsSystemId, GcsComponentId, _sequenceNumber++);

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, _sendIp);
        }

        public async Task CommandArm()
        {
            var sendpacket = _mavParse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,
            new MAVLink.mavlink_command_long_t()
            {
                target_system = AutopilotSystemId,
                target_component = AutopilotComponentId,
                command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM,
                confirmation = 0,
                param1 = 1,
                param2 = 0,
                param3 = 0,
                param4 = 0,
                param5 = 0,
                param6 = 0,
                param7 = 0,
            }, false, GcsSystemId, GcsComponentId, _sequenceNumber++);

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, _sendIp);
        }

        public async Task CommandDisarm()
        {
            var sendpacket = _mavParse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,
            new MAVLink.mavlink_command_long_t()
            {
                target_system = AutopilotSystemId,
                target_component = AutopilotComponentId,
                command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM,
                confirmation = 0,
                param1 = 0,
                param2 = 0,
                param3 = 0,
                param4 = 0,
                param5 = 0,
                param6 = 0,
                param7 = 0,
            }, false, GcsSystemId, GcsComponentId, _sequenceNumber++);

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, _sendIp);
        }

        public async Task SetMode(MAVLink.COPTER_MODE mode)
        {
            var sendpacket = _mavParse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.SET_MODE,
            new MAVLink.mavlink_set_mode_t()
            {
                base_mode = 1,
                custom_mode = (byte)mode,
                target_system = AutopilotSystemId,
            }, false, GcsSystemId, GcsComponentId, _sequenceNumber++);

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, _sendIp);
        }

        public async Task RequestDataStream()
        {
            var telemetryRequest = _mavParse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
            new MAVLink.mavlink_request_data_stream_t()
            {
                target_system = AutopilotSystemId,
                target_component = AutopilotComponentId,
                req_message_rate = 2,
                req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                start_stop = 1,
            }, false, GcsSystemId, GcsComponentId, _sequenceNumber++);

            await _udpClient.SendAsync(telemetryRequest, telemetryRequest.Length, _sendIp);
        }

        public async Task CommandLoiter(double latitudeRads, double longitudeRads, double altitudeMsl)
        {
            var latitudeDegrees = latitudeRads * 180.0 / Math.PI;
            var longitudeDegrees = longitudeRads * 180.0 / Math.PI;

            int latitudeDegE7 = Convert.ToInt32(latitudeDegrees * 1e7);
            int longitudeDegE7 = Convert.ToInt32(longitudeDegrees * 1e7);

            var sendpacket = _mavParse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.SET_POSITION_TARGET_GLOBAL_INT,
            new MAVLink.mavlink_set_position_target_global_int_t()
            {
                time_boot_ms = 0,
                target_system = AutopilotSystemId,
                target_component = AutopilotComponentId,
                afx = 0,
                afy = 0,
                afz = 0,
                alt = (float)altitudeMsl,
                coordinate_frame = 6,
                lat_int = latitudeDegE7,
                lon_int = longitudeDegE7,
                type_mask = 65528,
                vx = 0,
                vy = 0,
                vz = 0,
                yaw = 0,
                yaw_rate = 0
            }, false, GcsSystemId, GcsComponentId, _sequenceNumber++);

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, _sendIp);
        }

        public async Task ListenToMavlinkPackets()
        {
            await Task.Run(async () =>
            {
                while (true)
                {
                    var result = await _udpClient.ReceiveAsync();
                    var packet = _mavParse.ReadPacket(new MemoryStream(result.Buffer));

                    if (packet != null && packet.data != null)
                    {
                        _messageTypes.Add(packet.msgid);
                        HandleMavlinkPacket(packet);
                    }
                }
            });
        }

        private void HandleMavlinkPacket(MAVLink.MAVLinkMessage packet)
        {
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
    }
}
