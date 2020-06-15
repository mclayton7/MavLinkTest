using System.Collections.Generic;
using System.Threading.Tasks;

namespace App
{

    // See mission protocol: https://mavlink.io/en/services/mission.html
    class MavLinkMissionManager
    {
        IMavLinkService _mavLinkService;

        private readonly byte _autopilotSystemId = 1;
        private readonly byte _autopilotComponentId = 1;

        public MavLinkMissionManager(IMavLinkService mavLinkService)
        {
            _mavLinkService = mavLinkService;
        }

        public async Task CommandMission(IList<Location3d> points)
        {
            await SendMissionCount(points.Count, MAVLink.MAV_MISSION_TYPE.MISSION);
        }

        private async Task SendMissionCount(int numberOfMissionItems, MAVLink.MAV_MISSION_TYPE type)
        {
            var message = new MAVLink.mavlink_mission_count_t()
            {
                target_system = _autopilotSystemId,
                target_component = _autopilotComponentId,
                count = (ushort)numberOfMissionItems,
                mission_type = (byte)type
            
            };

            await _mavLinkService.SendMessage(MAVLink.MAVLINK_MSG_ID.MISSION_COUNT, message);
        }
    }
}