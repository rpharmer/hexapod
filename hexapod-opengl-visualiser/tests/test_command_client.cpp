#include "visualiser/net/udp_command_client.hpp"

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

bool Expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
  }
  return condition;
}

#ifndef _WIN32
std::vector<visualiser::net::CommandClientResult> WaitForReplies(
    visualiser::net::CommandClient& client) {
  for (int i = 0; i < 200; ++i) {
    auto replies = client.poll();
    if (!replies.empty()) {
      return replies;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return {};
}

bool SendReply(int socket_fd, const sockaddr_in& peer, socklen_t peer_len,
               const std::string& payload) {
  return ::sendto(socket_fd, payload.data(), payload.size(), 0,
                  reinterpret_cast<const sockaddr*>(&peer), peer_len) ==
         static_cast<ssize_t>(payload.size());
}
#endif

}  // namespace

int main() {
#ifdef _WIN32
  return 0;
#else
  const int server = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (!Expect(server >= 0, "create fake command server")) {
    return 1;
  }
  sockaddr_in bind_address{};
  bind_address.sin_family = AF_INET;
  bind_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  bind_address.sin_port = 0;
  if (!Expect(::bind(server, reinterpret_cast<sockaddr*>(&bind_address),
                     sizeof(bind_address)) == 0, "bind fake command server")) {
    ::close(server);
    return 1;
  }
  sockaddr_in server_address{};
  socklen_t server_len = sizeof(server_address);
  if (!Expect(::getsockname(server, reinterpret_cast<sockaddr*>(&server_address),
                            &server_len) == 0, "read fake command port")) {
    ::close(server);
    return 1;
  }
  timeval receive_timeout{};
  receive_timeout.tv_sec = 1;
  (void)::setsockopt(server, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout,
                     sizeof(receive_timeout));

  visualiser::net::CommandClient client({"127.0.0.1", ntohs(server_address.sin_port), 60});
  const auto begin = std::chrono::steady_clock::now();
  const auto submitted = client.scenarioList();
  const auto send_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - begin).count();
  if (!Expect(submitted.ok && submitted.ref == "viz-1" && send_ms < 150,
              "scenario.list submits without blocking for a reply")) {
    ::close(server);
    return 1;
  }
  char buffer[8192];
  sockaddr_in peer{};
  socklen_t peer_len = sizeof(peer);
  if (!Expect(::recvfrom(server, buffer, sizeof(buffer), 0,
                         reinterpret_cast<sockaddr*>(&peer), &peer_len) > 0,
              "fake server receives scenario.list")) {
    ::close(server);
    return 1;
  }
  if (!Expect(SendReply(server, peer, peer_len,
                        R"({"type":"command_result","schema_version":1,"ok":true,"ref":"viz-stale","reason":"listed"})"),
              "send stale reply")) {
    ::close(server);
    return 1;
  }
  if (!Expect(client.poll().empty(), "unmatched ref is ignored")) {
    ::close(server);
    return 1;
  }
  if (!Expect(SendReply(server, peer, peer_len,
                        R"({"type":"command_result","schema_version":1,"ok":true,"ref":"viz-1","reason":"listed","scenarios":["01_nominal"]})"),
              "send matched reply")) {
    ::close(server);
    return 1;
  }
  const auto listed = WaitForReplies(client);
  if (!Expect(listed.size() == 1 && listed[0].ok && listed[0].ref == "viz-1" &&
                  listed[0].request_type == "scenario.list" &&
                  listed[0].scenarios.size() == 1 && listed[0].scenarios[0] == "01_nominal",
              "matched reply carries the scenario list")) {
    ::close(server);
    return 1;
  }

  const auto run = client.scenarioRun("01_nominal");
  peer_len = sizeof(peer);
  if (!Expect(run.ok && ::recvfrom(server, buffer, sizeof(buffer), 0,
                                    reinterpret_cast<sockaddr*>(&peer), &peer_len) > 0,
              "send scenario.run")) {
    ::close(server);
    return 1;
  }
  if (!Expect(SendReply(server, peer, peer_len,
                        R"({"type":"command_result","schema_version":1,"ok":false,"ref":"viz-2","reason":"rejected"})"),
              "send apply rejection")) {
    ::close(server);
    return 1;
  }
  const auto rejected = WaitForReplies(client);
  if (!Expect(rejected.size() == 1 && !rejected[0].ok &&
                  rejected[0].request_type == "scenario.run" &&
                  rejected[0].reason == "rejected",
              "apply rejection is not confused with successful send")) {
    ::close(server);
    return 1;
  }

  const auto stop = client.scenarioStop();
  if (!Expect(stop.ok, "send scenario.stop for timeout test")) {
    ::close(server);
    return 1;
  }
  peer_len = sizeof(peer);
  if (!Expect(::recvfrom(server, buffer, sizeof(buffer), 0,
                         reinterpret_cast<sockaddr*>(&peer), &peer_len) > 0,
              "fake server receives scenario.stop")) {
    ::close(server);
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  const auto timed_out = client.poll();
  const bool timeout_ok = Expect(timed_out.size() == 1 && !timed_out[0].ok &&
                                     timed_out[0].ref == stop.ref &&
                                     timed_out[0].reason == "command_result timed out",
                                 "missing apply reply produces a correlated timeout");
  const auto stand = client.standHold(0.14);
  peer_len = sizeof(peer);
  const ssize_t stand_bytes = ::recvfrom(server, buffer, sizeof(buffer), 0,
                                         reinterpret_cast<sockaddr*>(&peer), &peer_len);
  const std::string stand_payload = stand_bytes > 0
      ? std::string(buffer, static_cast<std::size_t>(stand_bytes)) : std::string{};
  const bool stand_payload_ok = Expect(stand.ok &&
      stand_payload.find("\"mode\":\"STAND\"") != std::string::npos &&
      stand_payload.find("\"speed_mps\":0") != std::string::npos &&
      stand_payload.find("\"yaw_rate_radps\":0") != std::string::npos,
      "stand-and-hold sends a supported stationary mode");
  const bool stand_reply_sent = stand_payload_ok && SendReply(server, peer, peer_len,
      R"({"type":"command_result","schema_version":1,"ok":true,"ref":"viz-4","reason":"motion.set accepted"})");
  const auto stand_reply = stand_reply_sent ? WaitForReplies(client)
                                            : std::vector<visualiser::net::CommandClientResult>{};
  const bool stand_reply_ok = Expect(stand_reply.size() == 1 && stand_reply[0].ok &&
      stand_reply[0].ref == stand.ref && stand_reply[0].request_type == "motion.set",
      "stand-and-hold receives its own apply result");

  const auto empty_list = client.scenarioList();
  peer_len = sizeof(peer);
  const bool empty_list_sent = Expect(empty_list.ok &&
      ::recvfrom(server, buffer, sizeof(buffer), 0,
                 reinterpret_cast<sockaddr*>(&peer), &peer_len) > 0,
      "empty scenario list request sent");
  const bool empty_list_reply_sent = empty_list_sent && SendReply(server, peer, peer_len,
      std::string(R"({"type":"command_result","schema_version":1,"ok":true,"ref":")") +
      empty_list.ref + R"(","reason":"listed","scenarios":[]})");
  const auto empty_list_reply = empty_list_reply_sent ? WaitForReplies(client)
      : std::vector<visualiser::net::CommandClientResult>{};
  const bool empty_list_ok = Expect(empty_list_reply.size() == 1 &&
      empty_list_reply[0].scenarios.empty(), "empty list reply is accepted");

  const auto stop_after_empty = client.scenarioStop();
  peer_len = sizeof(peer);
  const bool stop_after_empty_sent = Expect(stop_after_empty.ok &&
      ::recvfrom(server, buffer, sizeof(buffer), 0,
                 reinterpret_cast<sockaddr*>(&peer), &peer_len) > 0,
      "scenario.stop remains sendable after an empty list");
  const bool stop_after_empty_reply_sent = stop_after_empty_sent && SendReply(server, peer, peer_len,
      std::string(R"({"type":"command_result","schema_version":1,"ok":true,"ref":")") +
      stop_after_empty.ref + R"(","reason":"stopped"})");
  const auto stop_after_empty_reply = stop_after_empty_reply_sent ? WaitForReplies(client)
      : std::vector<visualiser::net::CommandClientResult>{};
  const bool stop_after_empty_ok = Expect(stop_after_empty_reply.size() == 1 &&
      stop_after_empty_reply[0].ok && stop_after_empty_reply[0].ref == stop_after_empty.ref,
      "scenario.stop applies after an empty list");

  const auto missing_list = client.scenarioList();
  peer_len = sizeof(peer);
  const bool missing_list_sent = Expect(missing_list.ok &&
      ::recvfrom(server, buffer, sizeof(buffer), 0,
                 reinterpret_cast<sockaddr*>(&peer), &peer_len) > 0,
      "scenario.list request sent for timeout case");
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  const auto missing_list_timeout = client.poll();
  const bool missing_list_ok = Expect(missing_list_timeout.size() == 1 &&
      missing_list_timeout[0].ref == missing_list.ref && !missing_list_timeout[0].ok,
      "scenario.list timeout is correlated");
  const auto stop_after_timeout = client.scenarioStop();
  peer_len = sizeof(peer);
  const bool stop_after_timeout_sent = Expect(stop_after_timeout.ok &&
      ::recvfrom(server, buffer, sizeof(buffer), 0,
                 reinterpret_cast<sockaddr*>(&peer), &peer_len) > 0,
      "scenario.stop remains sendable after list timeout");
  ::close(server);
  return timeout_ok && stand_payload_ok && stand_reply_sent && stand_reply_ok &&
      empty_list_sent && empty_list_reply_sent && empty_list_ok &&
      stop_after_empty_sent && stop_after_empty_reply_sent && stop_after_empty_ok &&
      missing_list_sent && missing_list_ok && stop_after_timeout_sent ? 0 : 1;
#endif
}
