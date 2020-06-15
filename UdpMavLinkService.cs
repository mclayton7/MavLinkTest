using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;

namespace App
{
    class UdpMavLinkService : IMavLinkService
    {
        public event EventHandler<MAVLink.mavlink_heartbeat_t> HeartbeatReceived;
        public event EventHandler<MAVLink.mavlink_global_position_int_t> GlobalPositionIntReceived;
        public event EventHandler<MAVLink.mavlink_command_ack_t> CommandAckReceived;
        public event EventHandler<MAVLink.mavlink_statustext_t> StatusTextReceived;
        public event EventHandler<MAVLink.mavlink_mission_request_int_t> MissionRequestIntReceived;

        private readonly UdpClient _udpClient;
        private readonly IPEndPoint _sendIp;

        private readonly MAVLink.MavlinkParse _mavParse = new MAVLink.MavlinkParse();
        HashSet<uint> _messageTypes = new HashSet<uint>();

        private readonly byte _gcsSystemId = 255;
        private readonly byte _gcsComponentId = 190;

        private int _sequenceNumber = 0;

        public UdpMavLinkService(UdpClient udpClient, IPEndPoint sendIP)
        {
            _udpClient = udpClient;
            _sendIp = sendIP;
        }

        public async Task SendMessage(MAVLink.MAVLINK_MSG_ID messageId, object message)
        {
            var sendpacket = _mavParse.GenerateMAVLinkPacket20(
                messageId,
                message, 
                false, 
                _gcsSystemId, 
                _gcsComponentId, 
                _sequenceNumber++);

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
                        HeartbeatReceived?.Invoke(this, packet.ToStructure<MAVLink.mavlink_heartbeat_t>());
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
                        GlobalPositionIntReceived?.Invoke(this, packet.ToStructure<MAVLink.mavlink_global_position_int_t>());
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
                case (uint)MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST_INT:
                    {
                        MissionRequestIntReceived?.Invoke(this, packet.ToStructure<MAVLink.mavlink_mission_request_int_t>());
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
                        CommandAckReceived?.Invoke(this, packet.ToStructure<MAVLink.mavlink_command_ack_t>());
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
                        StatusTextReceived?.Invoke(this, packet.ToStructure<MAVLink.mavlink_statustext_t>());
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