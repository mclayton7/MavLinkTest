using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;

namespace App
{
    class MavLinkControl
    {
        private readonly UdpClient _udpClient;
        private readonly MAVLink.MavlinkParse _mavParse = new MAVLink.MavlinkParse();
        private int _sequenceNumber = 0;


        public byte GcsSystemId { get; } = 255;
        public byte GcsComponentId { get; } = 190;

        public byte AutopilotSystemId { get; } = 1;
        public byte AutopilotComponentId { get; } = 1;

        public MavLinkControl(UdpClient client)
        {
            _udpClient = client;
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

            Console.WriteLine($"Sending heartbeat");
            await _udpClient.SendAsync(sendpacket, sendpacket.Length, new IPEndPoint(IPAddress.Parse("239.255.145.50"), 14550));
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

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, new IPEndPoint(IPAddress.Parse("239.255.145.50"), 14550));
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

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, new IPEndPoint(IPAddress.Parse("239.255.145.50"), 14550));
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

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, new IPEndPoint(IPAddress.Parse("239.255.145.50"), 14550));
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

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, new IPEndPoint(IPAddress.Parse("239.255.145.50"), 14550));
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

            await _udpClient.SendAsync(telemetryRequest, telemetryRequest.Length, new IPEndPoint(IPAddress.Parse("239.255.145.50"), 14550));
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

            await _udpClient.SendAsync(sendpacket, sendpacket.Length, new IPEndPoint(IPAddress.Parse("239.255.145.50"), 14550));
        }
    }
}
