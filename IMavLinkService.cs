using System;
using System.Threading.Tasks;

namespace App
{
    interface IMavLinkService
    {
        event EventHandler<MAVLink.mavlink_heartbeat_t> HeartbeatReceived;
        event EventHandler<MAVLink.mavlink_global_position_int_t> GlobalPositionIntReceived;
        event EventHandler<MAVLink.mavlink_command_ack_t> CommandAckReceived;
        event EventHandler<MAVLink.mavlink_statustext_t> StatusTextReceived;
        event EventHandler<MAVLink.mavlink_mission_request_int_t> MissionRequestIntReceived;

        Task SendMessage(MAVLink.MAVLINK_MSG_ID messageId, object message);
    }
}