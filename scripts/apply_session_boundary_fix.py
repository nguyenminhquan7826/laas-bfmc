#!/usr/bin/env python3
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly 1 match, found {count}")
    return text.replace(old, new, 1)


client_path = Path("src/logical_robot/ParkingServerClient.cpp")
client = client_path.read_text()

old_close = '''    state_ = ConnectionState::DISCONNECTED;\n    rx_buffer_.clear();\n    tx_queue_.clear();\n    tx_offset_ = 0U;\n}\n\nvoid ParkingServerClient::markDisconnected()\n'''
new_close = '''    state_ = ConnectionState::DISCONNECTED;\n    rx_buffer_.clear();\n    rx_messages_.clear();\n    tx_queue_.clear();\n    tx_offset_ = 0U;\n}\n\nvoid ParkingServerClient::markDisconnected()\n'''
client = replace_once(client, old_close, new_close, "client close purge")

old_disconnect = '''    state_ = ConnectionState::DISCONNECTED;\n    rx_buffer_.clear();\n    tx_queue_.clear();\n    tx_offset_ = 0U;\n}\n\nvoid ParkingServerClient::beginConnect'''
new_disconnect = '''    state_ = ConnectionState::DISCONNECTED;\n    rx_buffer_.clear();\n    // Decoded messages are scoped to the TCP session that produced them.\n    // Never let an ACK/session snapshot from a dead connection survive into\n    // the next connection and overwrite Executive's fail-closed HOLD state.\n    rx_messages_.clear();\n    tx_queue_.clear();\n    tx_offset_ = 0U;\n}\n\nvoid ParkingServerClient::beginConnect'''
client = replace_once(client, old_disconnect, new_disconnect, "client disconnect purge")
client_path.write_text(client)

exec_path = Path("src/execution_control/Executive.cpp")
exec_text = exec_path.read_text()
old_exec = '''        std::cout << "[PARKING][SYNC] connected -> HOLD awaiting session\\n";\n    }\n\n    // Do not replay trajectory-scoped safety events yet. A reconnect may\n'''
new_exec = '''        std::cout << "[PARKING][SYNC] connected -> HOLD awaiting session\\n";\n    }\n\n    // A disconnected socket is a hard session boundary. ParkingServerClient\n    // purges decoded RX messages on disconnect; returning here is a second\n    // guard so no stale snapshot can overwrite SERVER_DISCONNECTED/HOLD.\n    if (!parking_server_connected_) {\n        return;\n    }\n\n    // Do not replay trajectory-scoped safety events yet. A reconnect may\n'''
exec_text = replace_once(exec_text, old_exec, new_exec, "Executive disconnect inbox guard")
exec_path.write_text(exec_text)

test_path = Path("server_v1/tests/test_parking_server_client.cpp")
test = test_path.read_text()
old_test = '''void testReconnectAfterPeerClose()\n{\n    ScriptedServer server([](int listen_fd) {\n        const int first = acceptClient(listen_fd);\n        ::close(first);\n\n        const int second = acceptClient(listen_fd);\n        sendAll(second,\n                "{\\\"type\\\":\\\"ack\\\",\\\"version\\\":1,\\\"accepted\\\":true,\\\"seq\\\":77}\\n");\n        std::this_thread::sleep_for(std::chrono::milliseconds(20));\n        ::close(second);\n    });\n\n    Config config = makeConfig(server.port());\n    ParkingServerClient client(config);\n    client.init();\n\n    const auto messages = collectMessages(client, 1, 2000);\n    server.joinAndRethrow();\n\n    CHECK_TRUE(messages.size() == 1, "reconnect_no_message");\n    CHECK_TRUE(messages[0].type == ParkingServerMessageType::ACK,\n               "reconnect_message_not_ack");\n    CHECK_TRUE(messages[0].has_sequence && messages[0].sequence == 77,\n               "reconnect_wrong_seq");\n    client.close();\n}\n'''
new_test = '''void testReconnectAfterPeerClose()\n{\n    ScriptedServer server([](int listen_fd) {\n        const int first = acceptClient(listen_fd);\n        sendAll(first,\n                "{\\\"type\\\":\\\"ack\\\",\\\"version\\\":1,\\\"accepted\\\":true,\\\"seq\\\":66}\\n");\n        ::shutdown(first, SHUT_RDWR);\n        ::close(first);\n\n        const int second = acceptClient(listen_fd);\n        sendAll(second,\n                "{\\\"type\\\":\\\"ack\\\",\\\"version\\\":1,\\\"accepted\\\":true,\\\"seq\\\":77}\\n");\n        std::this_thread::sleep_for(std::chrono::milliseconds(20));\n        ::close(second);\n    });\n\n    Config config = makeConfig(server.port());\n    ParkingServerClient client(config);\n    client.init();\n    CHECK_TRUE(waitConnected(client), "reconnect_first_connect_failed");\n\n    // Give the peer time to queue the stale ACK + FIN, then drive one session\n    // to DISCONNECTED. markDisconnected() must purge seq=66 before reconnect.\n    std::this_thread::sleep_for(std::chrono::milliseconds(20));\n    bool saw_disconnect = false;\n    const auto disconnect_deadline =\n        std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);\n    while (std::chrono::steady_clock::now() < disconnect_deadline) {\n        client.service();\n        if (!client.connected()) {\n            saw_disconnect = true;\n            break;\n        }\n        std::this_thread::sleep_for(std::chrono::milliseconds(1));\n    }\n    CHECK_TRUE(saw_disconnect, "reconnect_disconnect_not_observed");\n\n    ParkingServerMessage stale;\n    CHECK_TRUE(!client.popMessage(stale),\n               "reconnect_stale_message_survived_session_boundary");\n\n    CHECK_TRUE(waitConnected(client, 1500), "reconnect_second_connect_failed");\n    const auto messages = collectMessages(client, 1, 1500);\n    server.joinAndRethrow();\n\n    CHECK_TRUE(messages.size() == 1, "reconnect_no_message");\n    CHECK_TRUE(messages[0].type == ParkingServerMessageType::ACK,\n               "reconnect_message_not_ack");\n    CHECK_TRUE(messages[0].has_sequence && messages[0].sequence == 77,\n               "reconnect_wrong_seq_or_stale_seq66");\n    client.close();\n}\n'''
test = replace_once(test, old_test, new_test, "reconnect stale RX regression")
test_path.write_text(test)

print("[PASS] session-boundary stale RX patch applied")
