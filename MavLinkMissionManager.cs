using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace App
{

    // See mission protocol: https://mavlink.io/en/services/mission.html
    class MavLinkMissionManager
    {
        private readonly IMavLinkService _mavLinkService;

        private readonly System.Timers.Timer _heartbeatTimer = new System.Timers.Timer(1000);

        public MavLinkMissionManager(IMavLinkService mavLinkService)
        {
            _mavLinkService = mavLinkService;
            _mavLinkService.MissionRequestIntReceived += (sender, packet) => HandleMissionRequestIntReceived(packet);

            _heartbeatTimer.AutoReset = false;
            _heartbeatTimer.Elapsed += (sender, args) => ResetMissionState();
        }

        public async Task CommandMission(IList<Location3d> points)
        {
            await SendMissionCount(points.Count, MAVLink.MAV_MISSION_TYPE.MISSION);
        }

        private void ResetMissionState()
        {
            Console.WriteLine("Resetting Mission state");
        }



        private void HandleMissionRequestIntReceived(MAVLink.mavlink_mission_request_int_t message)
        {

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
            _heartbeatTimer.Start();
        }

        private async Task SendMissionItemInt(MAVLink.MAV_MISSION_TYPE type)
        {
            var message = new MAVLink.mavlink_mission_item_int_t()
            {
                target_system = MavLinkConstants.AUTOPILOT_SYSTEM_ID,
                target_component = MavLinkConstants.AUTOPILOT_COMPONENT_ID,
                frame = (byte)MAVLink.MAV_FRAME.GLOBAL,
                // command = MAVLink.MAV_CMD.
                mission_type = (byte)type
            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.MISSION_COUNT, message);
            _heartbeatTimer.Start();
        }
    }
}