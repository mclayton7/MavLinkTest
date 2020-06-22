using MavLinkTest;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace App
{
    // See mission protocol: https://mavlink.io/en/services/mission.html
    class MavLinkMissionManager
    {
        private readonly IMavLinkService _mavLinkService;
        private readonly System.Timers.Timer _missionHeartbeatTimer = new System.Timers.Timer(5000);

        private IList<Location3d> _points = new List<Location3d>();
        private int _waypointNumber = 0;

        public MavLinkMissionManager(IMavLinkService mavLinkService)
        {
            _mavLinkService = mavLinkService;
            _mavLinkService.MissionRequestIntReceived += async (sender, packet) => await HandleMissionRequestIntReceived(packet);

            _missionHeartbeatTimer.AutoReset = false;
            _missionHeartbeatTimer.Elapsed += (sender, args) => ResetMissionState();
        }

        public async Task CommandMission(IList<Location3d> points)
        {
            ResetMissionState();

            _points = points;
            await SendMissionCount(points.Count, MAVLink.MAV_MISSION_TYPE.MISSION);
            _missionHeartbeatTimer.Start();
        }

        private void ResetMissionState()
        {
            Console.WriteLine("=== Resetting Mission state");
            _waypointNumber = 0;
            _points.Clear();
        }

        private async Task HandleMissionRequestIntReceived(MAVLink.mavlink_mission_request_int_t message)
        {
            var nextWaypoint = _points[_waypointNumber];
            await SendMissionItemInt(MAVLink.MAV_MISSION_TYPE.MISSION, nextWaypoint);
            _missionHeartbeatTimer.Start();
        }

        private async Task SendMissionCount(int numberOfMissionItems, MAVLink.MAV_MISSION_TYPE type)
        {
            var message = new MAVLink.mavlink_mission_count_t()
            {
                target_system = MavLinkConstants.AUTOPILOT_SYSTEM_ID,
                target_component = MavLinkConstants.AUTOPILOT_COMPONENT_ID,
                count = (ushort)numberOfMissionItems,
                mission_type = (byte)type

            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.MISSION_COUNT, message);
        }

        private async Task SendMissionItemInt(MAVLink.MAV_MISSION_TYPE type, Location3d location)
        {
            var holdTime = 0;
            var acceptRadius = 1;
            var passRadius = 0;
            var yaw = float.NaN;

            var message = new MAVLink.mavlink_mission_item_int_t()
            {
                target_system = MavLinkConstants.AUTOPILOT_SYSTEM_ID,
                target_component = MavLinkConstants.AUTOPILOT_COMPONENT_ID,
                frame = (byte)MAVLink.MAV_FRAME.GLOBAL,
                command = (ushort)MAVLink.MAV_CMD.WAYPOINT,
                param1 = holdTime,
                param2 = acceptRadius,
                param3 = passRadius,
                param4 = yaw,
                x = MavLinkUtilities.ConvertRadiansToDegreesE7(location.LatitudeInRadians),
                y = MavLinkUtilities.ConvertRadiansToDegreesE7(location.LongitudeInRadians),
                z = (float)location.AltitudeMslInMeters,
                mission_type = (byte)type,
            };
            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.MISSION_ITEM_INT, message);
            _missionHeartbeatTimer.Start();
        }
    }
}