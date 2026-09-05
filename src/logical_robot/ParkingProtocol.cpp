#include "ParkingProtocol.hpp"

#include <cmath>
#include <limits>
#include <set>

#include <json-c/json.h>

#include "../laas_core/Time.hpp"

namespace laas {

namespace {

bool finite(double value)
{
    return std::isfinite(value);
}

const char* slotStateToString(ParkingSlotState state)
{
    switch (state) {
    case ParkingSlotState::FREE: return "FREE";
    case ParkingSlotState::OCCUPIED: return "OCCUPIED";
    case ParkingSlotState::UNKNOWN:
    default: return "UNKNOWN";
    }
}

bool parseDirection(const std::string& value, MotionDirection& out)
{
    if (value == "FORWARD") {
        out = MotionDirection::FORWARD;
        return true;
    }
    if (value == "REVERSE") {
        out = MotionDirection::REVERSE;
        return true;
    }
    return false;
}

std::string dumpJson(json_object* object)
{
    const char* text = json_object_to_json_string_ext(
        object, JSON_C_TO_STRING_PLAIN);
    return text != nullptr ? std::string(text) : std::string();
}

bool getField(json_object* object, const char* key, json_object*& out)
{
    out = nullptr;
    return object != nullptr &&
           json_object_object_get_ex(object, key, &out) != 0 &&
           out != nullptr;
}

bool getString(json_object* object, const char* key, std::string& out)
{
    json_object* value = nullptr;
    if (!getField(object, key, value) ||
        !json_object_is_type(value, json_type_string)) {
        return false;
    }
    const char* text = json_object_get_string(value);
    if (text == nullptr) {
        return false;
    }
    out = text;
    return true;
}

bool getBool(json_object* object, const char* key, bool& out)
{
    json_object* value = nullptr;
    if (!getField(object, key, value) ||
        !json_object_is_type(value, json_type_boolean)) {
        return false;
    }
    out = json_object_get_boolean(value) != 0;
    return true;
}

bool getUnsigned(json_object* object, const char* key, std::uint64_t& out)
{
    json_object* value = nullptr;
    if (!getField(object, key, value) ||
        !json_object_is_type(value, json_type_int)) {
        return false;
    }
    const std::int64_t signed_value = json_object_get_int64(value);
    if (signed_value < 0) {
        return false;
    }
    out = static_cast<std::uint64_t>(signed_value);
    return true;
}

bool getFiniteNumber(json_object* object, const char* key, double& out)
{
    json_object* value = nullptr;
    if (!getField(object, key, value) ||
        !(json_object_is_type(value, json_type_double) ||
          json_object_is_type(value, json_type_int))) {
        return false;
    }
    out = json_object_get_double(value);
    return finite(out);
}

bool parseSession(json_object* object, ParkingSessionSnapshot& out)
{
    if (object == nullptr || !json_object_is_type(object, json_type_object)) {
        return false;
    }

    if (!getUnsigned(object, "session_id", out.session_id) ||
        !getString(object, "state", out.state)) {
        return false;
    }

    json_object* tid = nullptr;
    if (getField(object, "active_trajectory_id", tid) &&
        !json_object_is_type(tid, json_type_null)) {
        if (!json_object_is_type(tid, json_type_int)) {
            return false;
        }
        const std::int64_t value = json_object_get_int64(tid);
        if (value < 0) {
            return false;
        }
        out.active_trajectory_id = static_cast<std::uint64_t>(value);
        out.has_active_trajectory = true;
    }

    json_object* target = nullptr;
    if (getField(object, "target_slot", target) &&
        !json_object_is_type(target, json_type_null)) {
        if (!json_object_is_type(target, json_type_string)) {
            return false;
        }
        const char* text = json_object_get_string(target);
        if (text == nullptr) {
            return false;
        }
        out.target_slot = text;
    }

    json_object* replans = nullptr;
    if (getField(object, "replan_count", replans)) {
        if (!json_object_is_type(replans, json_type_int)) {
            return false;
        }
        const std::int64_t value = json_object_get_int64(replans);
        if (value < 0) {
            return false;
        }
        out.replan_count = static_cast<std::uint64_t>(value);
    }
    return true;
}

bool checkVersion(json_object* object, std::uint32_t& version)
{
    std::uint64_t value = 0;
    if (!getUnsigned(object, "version", value) ||
        value != ParkingProtocol::kVersion) {
        return false;
    }
    version = static_cast<std::uint32_t>(value);
    return true;
}

json_object* makeBase(const char* type)
{
    json_object* object = json_object_new_object();
    json_object_object_add(object, "type", json_object_new_string(type));
    json_object_object_add(object, "version",
                           json_object_new_int64(ParkingProtocol::kVersion));
    return object;
}

void addUnsigned(json_object* object, const char* key, std::uint64_t value)
{
    json_object_object_add(object, key,
                           json_object_new_int64(static_cast<std::int64_t>(value)));
}

}  // namespace

bool ParkingProtocol::encodeVehiclePose(const VehiclePoseMsg& msg,
                                        std::string& line,
                                        std::string& reason)
{
    if (!msg.header.valid || msg.map_id.empty() || msg.source.empty() ||
        !finite(msg.x_m) || !finite(msg.y_m) || !finite(msg.yaw_rad)) {
        reason = "invalid_vehicle_pose";
        return false;
    }

    json_object* object = makeBase("vehicle_pose");
    addUnsigned(object, "seq", msg.sequence);
    addUnsigned(object, "timestamp_ms", msg.header.timestamp_ms);
    json_object_object_add(object, "map_id",
                           json_object_new_string(msg.map_id.c_str()));
    json_object_object_add(object, "source",
                           json_object_new_string(msg.source.c_str()));

    json_object* pose = json_object_new_object();
    json_object_object_add(pose, "x_m", json_object_new_double(msg.x_m));
    json_object_object_add(pose, "y_m", json_object_new_double(msg.y_m));
    json_object_object_add(pose, "yaw_rad", json_object_new_double(msg.yaw_rad));
    json_object_object_add(object, "pose", pose);

    line = dumpJson(object);
    json_object_put(object);
    if (line.empty()) {
        reason = "json_encode_failed";
        return false;
    }
    reason = "ok";
    return true;
}

bool ParkingProtocol::encodeParkingStatus(const ParkingStatusMsg& msg,
                                          std::string& line,
                                          std::string& reason)
{
    if (!msg.header.valid || msg.map_id.empty() || msg.slots.empty()) {
        reason = "invalid_parking_status";
        return false;
    }

    std::set<std::string> ids;
    json_object* slots = json_object_new_array();
    for (const auto& slot : msg.slots) {
        if (slot.id.empty() || !std::isfinite(slot.confidence) ||
            slot.confidence < 0.0F || slot.confidence > 1.0F ||
            !ids.insert(slot.id).second) {
            json_object_put(slots);
            reason = "invalid_parking_slot";
            return false;
        }
        json_object* item = json_object_new_object();
        json_object_object_add(item, "id", json_object_new_string(slot.id.c_str()));
        json_object_object_add(item, "state",
                               json_object_new_string(slotStateToString(slot.state)));
        json_object_object_add(item, "confidence",
                               json_object_new_double(slot.confidence));
        json_object_array_add(slots, item);
    }

    json_object* objects = json_object_new_array();
    for (const auto& evidence : msg.objects) {
        if (evidence.class_name.empty() || !std::isfinite(evidence.confidence) ||
            evidence.confidence < 0.0F || evidence.confidence > 1.0F ||
            !finite(evidence.relative_x_m) || !finite(evidence.relative_y_m)) {
            json_object_put(slots);
            json_object_put(objects);
            reason = "invalid_parking_object";
            return false;
        }
        json_object* item = json_object_new_object();
        json_object_object_add(item, "class",
                               json_object_new_string(evidence.class_name.c_str()));
        json_object_object_add(item, "confidence",
                               json_object_new_double(evidence.confidence));
        json_object_object_add(item, "relative_x_m",
                               json_object_new_double(evidence.relative_x_m));
        json_object_object_add(item, "relative_y_m",
                               json_object_new_double(evidence.relative_y_m));
        if (!evidence.associated_slot.empty()) {
            json_object_object_add(item, "associated_slot",
                                   json_object_new_string(evidence.associated_slot.c_str()));
        }
        json_object_array_add(objects, item);
    }

    json_object* object = makeBase("parking_status");
    addUnsigned(object, "seq", msg.sequence);
    addUnsigned(object, "timestamp_ms", msg.header.timestamp_ms);
    json_object_object_add(object, "map_id",
                           json_object_new_string(msg.map_id.c_str()));
    json_object_object_add(object, "slots", slots);
    json_object_object_add(object, "objects", objects);

    line = dumpJson(object);
    json_object_put(object);
    if (line.empty()) {
        reason = "json_encode_failed";
        return false;
    }
    reason = "ok";
    return true;
}

bool ParkingProtocol::encodePlanRequest(std::uint64_t sequence,
                                        std::uint64_t timestamp_ms,
                                        const std::string& map_id,
                                        bool new_session,
                                        std::string& line,
                                        std::string& reason)
{
    if (map_id.empty()) {
        reason = "invalid_map_id";
        return false;
    }
    json_object* object = makeBase("plan_request");
    addUnsigned(object, "seq", sequence);
    addUnsigned(object, "timestamp_ms", timestamp_ms);
    json_object_object_add(object, "map_id", json_object_new_string(map_id.c_str()));
    json_object_object_add(object, "new_session",
                           json_object_new_boolean(new_session ? 1 : 0));
    line = dumpJson(object);
    json_object_put(object);
    reason = line.empty() ? "json_encode_failed" : "ok";
    return !line.empty();
}

bool ParkingProtocol::encodeSafetyEvent(std::uint64_t timestamp_ms,
                                        std::uint64_t trajectory_id,
                                        bool has_trajectory_id,
                                        const std::string& event,
                                        std::string& line,
                                        std::string& reason)
{
    static const std::set<std::string> allowed = {
        "PEDESTRIAN_BLOCKING", "CRITICAL_OBSTACLE", "SERVER_TIMEOUT",
        "TRAJECTORY_INVALID", "SAFETY_CLEARED"
    };
    if (allowed.count(event) == 0U) {
        reason = "invalid_safety_event";
        return false;
    }
    json_object* object = makeBase("safety_event");
    addUnsigned(object, "timestamp_ms", timestamp_ms);
    json_object_object_add(object, "event", json_object_new_string(event.c_str()));
    if (has_trajectory_id) {
        addUnsigned(object, "trajectory_id", trajectory_id);
    }
    line = dumpJson(object);
    json_object_put(object);
    reason = line.empty() ? "json_encode_failed" : "ok";
    return !line.empty();
}

bool ParkingProtocol::encodeTrajectoryStatus(std::uint64_t sequence,
                                             std::uint64_t timestamp_ms,
                                             std::uint64_t trajectory_id,
                                             const std::string& status,
                                             const std::string& reason_text,
                                             std::string& line,
                                             std::string& reason)
{
    static const std::set<std::string> allowed = {
        "RECEIVED", "EXECUTING", "PAUSED", "COMPLETED", "REJECTED",
        "REPLAN_REQUESTED"
    };
    if (allowed.count(status) == 0U) {
        reason = "invalid_trajectory_status";
        return false;
    }
    json_object* object = makeBase("trajectory_status");
    addUnsigned(object, "seq", sequence);
    addUnsigned(object, "timestamp_ms", timestamp_ms);
    addUnsigned(object, "trajectory_id", trajectory_id);
    json_object_object_add(object, "status", json_object_new_string(status.c_str()));
    if (!reason_text.empty()) {
        json_object_object_add(object, "reason",
                               json_object_new_string(reason_text.c_str()));
    }
    line = dumpJson(object);
    json_object_put(object);
    reason = line.empty() ? "json_encode_failed" : "ok";
    return !line.empty();
}

bool ParkingProtocol::encodeSessionQuery(const std::string& map_id,
                                         std::string& line,
                                         std::string& reason)
{
    if (map_id.empty()) {
        reason = "invalid_map_id";
        return false;
    }
    json_object* object = makeBase("session_query");
    json_object_object_add(object, "map_id", json_object_new_string(map_id.c_str()));
    line = dumpJson(object);
    json_object_put(object);
    reason = line.empty() ? "json_encode_failed" : "ok";
    return !line.empty();
}

bool ParkingProtocol::decodeServerLine(const std::string& line,
                                       const std::string& expected_map_id,
                                       ParkingServerMessage& out,
                                       std::string& reason)
{
    out = ParkingServerMessage{};

    json_tokener* tokener = json_tokener_new();
    if (tokener == nullptr) {
        reason = "json_tokener_alloc_failed";
        return false;
    }
    json_object* object = json_tokener_parse_ex(tokener, line.c_str(),
                                                static_cast<int>(line.size()));
    const json_tokener_error error = json_tokener_get_error(tokener);
    json_tokener_free(tokener);
    if (error != json_tokener_success || object == nullptr ||
        !json_object_is_type(object, json_type_object)) {
        if (object != nullptr) {
            json_object_put(object);
        }
        reason = "invalid_json";
        return false;
    }

    std::uint32_t version = 0;
    std::string type;
    if (!checkVersion(object, version) || !getString(object, "type", type)) {
        json_object_put(object);
        reason = "invalid_common_fields";
        return false;
    }

    out.header.valid = true;
    out.header.timestamp_ms = nowMs();
    out.protocol_version = version;

    json_object* session_object = nullptr;
    if (getField(object, "session", session_object) &&
        !json_object_is_type(session_object, json_type_null)) {
        if (!parseSession(session_object, out.session)) {
            json_object_put(object);
            reason = "invalid_session";
            return false;
        }
        out.has_session = true;
    }

    if (type == "ack") {
        out.type = ParkingServerMessageType::ACK;
        if (!getBool(object, "accepted", out.accepted)) {
            json_object_put(object);
            reason = "invalid_ack";
            return false;
        }
        json_object* seq = nullptr;
        if (getField(object, "seq", seq) && !json_object_is_type(seq, json_type_null)) {
            if (!getUnsigned(object, "seq", out.sequence)) {
                json_object_put(object);
                reason = "invalid_ack_seq";
                return false;
            }
            out.has_sequence = true;
        }
        getString(object, "reason", out.reason);
        json_object_put(object);
        reason = "ok";
        return true;
    }

    if (type == "error") {
        out.type = ParkingServerMessageType::ERROR;
        getString(object, "reason", out.reason);
        json_object_put(object);
        reason = "ok";
        return true;
    }

    if (!getString(object, "map_id", out.map_id)) {
        json_object_put(object);
        reason = "missing_map_id";
        return false;
    }
    if (!expected_map_id.empty() && out.map_id != expected_map_id) {
        json_object_put(object);
        reason = "map_id_mismatch";
        return false;
    }

    if (type == "planning_result") {
        out.type = ParkingServerMessageType::PLANNING_RESULT;
        if (!getString(object, "status", out.status) ||
            !getString(object, "reason", out.reason)) {
            json_object_put(object);
            reason = "invalid_planning_result";
            return false;
        }
        json_object* source = nullptr;
        if (getField(object, "source_seq", source) &&
            !json_object_is_type(source, json_type_null)) {
            if (!getUnsigned(object, "source_seq", out.source_seq)) {
                json_object_put(object);
                reason = "invalid_source_seq";
                return false;
            }
            out.has_source_seq = true;
        }
        json_object_put(object);
        reason = "ok";
        return true;
    }

    if (type == "session_status") {
        out.type = ParkingServerMessageType::SESSION_STATUS;
        if (!out.has_session) {
            json_object_put(object);
            reason = "missing_session";
            return false;
        }
        json_object_put(object);
        reason = "ok";
        return true;
    }

    if (type != "trajectory") {
        json_object_put(object);
        reason = "unsupported_server_message_type";
        return false;
    }

    out.type = ParkingServerMessageType::TRAJECTORY;
    ParkingTrajectoryMsg trajectory;
    trajectory.header.valid = true;
    trajectory.header.timestamp_ms = out.header.timestamp_ms;
    trajectory.protocol_version = version;

    if (!getUnsigned(object, "trajectory_id", trajectory.trajectory_id) ||
        !getUnsigned(object, "source_seq", trajectory.source_seq) ||
        !getString(object, "target_slot", trajectory.target_slot) ||
        !getString(object, "reference_point", trajectory.reference_point) ||
        !getString(object, "goal_mode", trajectory.goal_mode) ||
        !getString(object, "validation", trajectory.validation) ||
        !getString(object, "prototype_warning", trajectory.prototype_warning)) {
        json_object_put(object);
        reason = "invalid_trajectory_fields";
        return false;
    }
    trajectory.map_id = out.map_id;

    if (trajectory.reference_point != "rear_axle_center" ||
        trajectory.validation != "PASS") {
        json_object_put(object);
        reason = "trajectory_contract_mismatch";
        return false;
    }

    json_object* points = nullptr;
    if (!getField(object, "points", points) ||
        !json_object_is_type(points, json_type_array) ||
        json_object_array_length(points) == 0U) {
        json_object_put(object);
        reason = "invalid_trajectory_points";
        return false;
    }

    const std::size_t count = json_object_array_length(points);
    trajectory.points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        json_object* point = json_object_array_get_idx(points, i);
        if (point == nullptr || !json_object_is_type(point, json_type_object)) {
            json_object_put(object);
            reason = "trajectory_point_not_object";
            return false;
        }
        double x = 0.0, y = 0.0, yaw = 0.0, speed = 0.0;
        std::string direction_text;
        if (!getFiniteNumber(point, "x_m", x) ||
            !getFiniteNumber(point, "y_m", y) ||
            !getFiniteNumber(point, "yaw_rad", yaw) ||
            !getFiniteNumber(point, "v_ref_mps", speed) ||
            !getString(point, "direction", direction_text)) {
            json_object_put(object);
            reason = "invalid_trajectory_point_fields";
            return false;
        }
        MotionDirection direction;
        if (!parseDirection(direction_text, direction) ||
            speed < -static_cast<double>(std::numeric_limits<float>::max()) ||
            speed > static_cast<double>(std::numeric_limits<float>::max()) ||
            (direction == MotionDirection::FORWARD && speed < -1e-9) ||
            (direction == MotionDirection::REVERSE && speed > 1e-9)) {
            json_object_put(object);
            reason = "trajectory_direction_or_speed_invalid";
            return false;
        }
        ParkingTrajectoryPoint parsed;
        parsed.x_m = x;
        parsed.y_m = y;
        parsed.yaw_rad = yaw;
        parsed.v_ref_mps = static_cast<float>(speed);
        parsed.direction = direction;
        trajectory.points.push_back(parsed);
    }

    out.trajectory = std::move(trajectory);
    json_object_put(object);
    reason = "ok";
    return true;
}

}  // namespace laas
