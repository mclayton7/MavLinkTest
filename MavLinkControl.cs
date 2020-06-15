using System;
using System.Threading.Tasks;

namespace App
{
    class MavLinkControl
    {
        private readonly IMavLinkService _mavLinkService;
        private readonly System.Timers.Timer _heartbeatTimer = new System.Timers.Timer(1000);

        public MavLinkControl(IMavLinkService mavLinkService)
        {
            _mavLinkService = mavLinkService;

            _heartbeatTimer.AutoReset = true;
            _heartbeatTimer.Elapsed += async (o, e) =>
            {
                await SendHeartbeat();
            };

            _heartbeatTimer.Start();

            mavLinkService.CommandAckReceived += (sender, packet) => 
            {
                Console.WriteLine($"Got an ACK from command. Result: {packet.result}");
            };

            mavLinkService.GlobalPositionIntReceived += (sender, packet) => 
            {
                double latitudeDegrees = packet.lat / (double)Math.Pow(10, 7);
                double longitudeDegrees = packet.lon / (double)Math.Pow(10, 7);
                double altMslMeters = packet.alt / (double)1000;
                Console.WriteLine($"{latitudeDegrees} {longitudeDegrees} {altMslMeters}");
            };
        }

        public async Task SendHeartbeat()
        {
            var message = new MAVLink.mavlink_heartbeat_t()
            {
                autopilot = (byte)MAVLink.MAV_AUTOPILOT.INVALID,
                base_mode = 0,
                custom_mode = 0,
                mavlink_version = 3,
                system_status = 0,
                type = (byte)MAVLink.MAV_TYPE.GCS
            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, message);
        }

        public async Task CommandTakeOff()
        {
            var message = new MAVLink.mavlink_command_long_t()
            {
                target_system = MavLinkConstants.AUTOPILOT_SYSTEM_ID,
                target_component = MavLinkConstants.AUTOPILOT_COMPONENT_ID,
                command = (ushort)MAVLink.MAV_CMD.TAKEOFF,
                param1 = 0,
                param2 = 0,
                param3 = 0,
                param4 = 0,
                param5 = 0,
                param6 = 0,
                param7 = 40,
            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, message);
        }

        public async Task CommandArm()
        {
            var message = new MAVLink.mavlink_command_long_t()
            {
                target_system = MavLinkConstants.AUTOPILOT_SYSTEM_ID,
                target_component = MavLinkConstants.AUTOPILOT_COMPONENT_ID,
                command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM,
                confirmation = 0,
                param1 = 1,
                param2 = 0,
                param3 = 0,
                param4 = 0,
                param5 = 0,
                param6 = 0,
                param7 = 0,
            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, message);
        }

        public async Task CommandDisarm()
        {
            var message = new MAVLink.mavlink_command_long_t()
            {
                target_system = MavLinkConstants.AUTOPILOT_SYSTEM_ID,
                target_component = MavLinkConstants.AUTOPILOT_COMPONENT_ID,
                command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM,
                confirmation = 0,
                param1 = 0,
                param2 = 0,
                param3 = 0,
                param4 = 0,
                param5 = 0,
                param6 = 0,
                param7 = 0,
            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, message);
        }

        public async Task SetMode(MAVLink.COPTER_MODE mode)
        {
            var message = new MAVLink.mavlink_set_mode_t()
            {
                base_mode = 1,
                custom_mode = (byte)mode,
                target_system = MavLinkConstants.AUTOPILOT_SYSTEM_ID,
            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.SET_MODE, message);
        }

        public async Task RequestDataStream()
        {
            var message = new MAVLink.mavlink_request_data_stream_t()
            {
                target_system = MavLinkConstants.AUTOPILOT_SYSTEM_ID,
                target_component = MavLinkConstants.AUTOPILOT_COMPONENT_ID,
                req_message_rate = 2,
                req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                start_stop = 1,
            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM, message);
        }

        public async Task CommandLoiter(Location3d location)
        {
            var latitudeDegrees = location.LatitudeInRadians * 180.0 / Math.PI;
            var longitudeDegrees = location.LongitudeInRadians * 180.0 / Math.PI;

            int latitudeDegE7 = Convert.ToInt32(latitudeDegrees * 1e7);
            int longitudeDegE7 = Convert.ToInt32(longitudeDegrees * 1e7);

            var message = new MAVLink.mavlink_set_position_target_global_int_t()
            {
                time_boot_ms = 0,
                target_system = MavLinkConstants.AUTOPILOT_SYSTEM_ID,
                target_component = MavLinkConstants.AUTOPILOT_COMPONENT_ID,
                afx = 0,
                afy = 0,
                afz = 0,
                alt = (float)location.AltitudeMslInMeters,
                coordinate_frame = 6,
                lat_int = latitudeDegE7,
                lon_int = longitudeDegE7,
                type_mask = 65528,
                vx = 0,
                vy = 0,
                vz = 0,
                yaw = 0,
                yaw_rate = 0
            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.SET_POSITION_TARGET_GLOBAL_INT, message);
        }
    }
}
