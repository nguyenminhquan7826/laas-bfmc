#include "LocalParkingPlannerWorker.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <utility>

#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace laas {

LocalParkingPlannerWorker::LocalParkingPlannerWorker(const Config& config)
    : config_(config)
{
}

LocalParkingPlannerWorker::~LocalParkingPlannerWorker()
{
    stop();
}

bool LocalParkingPlannerWorker::start(
    std::uint64_t decision_id,
    const std::string& request_line,
    std::string& reason)
{
    if (decision_id == 0U || request_line.empty()) {
        reason = "invalid_local_planner_request";
        return false;
    }
    if (!config_.parking.enable_local_parking_planner) {
        reason = "local_planner_disabled";
        return false;
    }
    if (running_.load()) {
        reason = "local_planner_busy";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (result_ready_ || thread_.joinable()) {
            reason = "previous_local_planner_result_not_collected";
            return false;
        }
    }

    stop_requested_.store(false);
    running_.store(true);
    try {
        thread_ = std::thread(
            [this, decision_id, request_line]() {
                LocalParkingPlannerProcessResult result;
                try {
                    result = runProcess(decision_id, request_line);
                } catch (const std::exception& error) {
                    result.decision_id = decision_id;
                    result.reason =
                        std::string("local_planner_exception:") + error.what();
                } catch (...) {
                    result.decision_id = decision_id;
                    result.reason = "local_planner_unknown_exception";
                }
                {
                    std::lock_guard<std::mutex> lock(result_mutex_);
                    result_ = std::move(result);
                    result_ready_ = true;
                }
                running_.store(false);
            });
    } catch (const std::exception& error) {
        running_.store(false);
        reason = std::string("local_planner_thread_start_failed:") + error.what();
        return false;
    }

    reason = "ok";
    return true;
}

bool LocalParkingPlannerWorker::poll(
    LocalParkingPlannerProcessResult& result)
{
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (!result_ready_) {
            return false;
        }
        result = std::move(result_);
        result_ = LocalParkingPlannerProcessResult{};
        result_ready_ = false;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    return true;
}

void LocalParkingPlannerWorker::stop()
{
    stop_requested_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false);
    std::lock_guard<std::mutex> lock(result_mutex_);
    result_ready_ = false;
    result_ = LocalParkingPlannerProcessResult{};
}

LocalParkingPlannerProcessResult LocalParkingPlannerWorker::runProcess(
    std::uint64_t decision_id,
    const std::string& request_line)
{
    LocalParkingPlannerProcessResult result;
    result.decision_id = decision_id;

    int sockets[2] = {-1, -1};
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0) {
        result.reason =
            std::string("local_planner_socketpair_failed:") +
            std::strerror(errno);
        return result;
    }

    const pid_t pid = ::fork();
    if (pid < 0) {
        result.reason =
            std::string("local_planner_fork_failed:") + std::strerror(errno);
        ::close(sockets[0]);
        ::close(sockets[1]);
        return result;
    }

    if (pid == 0) {
        ::close(sockets[0]);
        if (::dup2(sockets[1], STDIN_FILENO) < 0 ||
            ::dup2(sockets[1], STDOUT_FILENO) < 0) {
            _exit(126);
        }
        ::close(sockets[1]);
        ::execlp(
            config_.parking.local_planner_python.c_str(),
            config_.parking.local_planner_python.c_str(),
            config_.parking.local_planner_script.c_str(),
            "--root",
            config_.parking.local_planner_root.c_str(),
            "--protocol-output",
            static_cast<char*>(nullptr));
        _exit(127);
    }

    ::close(sockets[1]);
    const int fd = sockets[0];
    const std::string wire_request = request_line + "\n";
    std::size_t offset = 0U;
    while (offset < wire_request.size()) {
        const ssize_t sent = ::send(
            fd, wire_request.data() + offset,
            wire_request.size() - offset, MSG_NOSIGNAL);
        if (sent > 0) {
            offset += static_cast<std::size_t>(sent);
            continue;
        }
        if (sent < 0 && errno == EINTR) {
            continue;
        }
        result.reason =
            std::string("local_planner_request_write_failed:") +
            std::strerror(errno);
        break;
    }
    ::shutdown(fd, SHUT_WR);

    const auto start = std::chrono::steady_clock::now();
    const int timeout_ms =
        config_.parking.local_planner_timeout_ms > 0
            ? config_.parking.local_planner_timeout_ms
            : 1;
    const std::size_t max_output =
        config_.parking.local_planner_max_output_bytes > 0
            ? static_cast<std::size_t>(
                  config_.parking.local_planner_max_output_bytes)
            : 1U;
    bool child_exited = false;
    int child_status = 0;

    while (!child_exited) {
        if (stop_requested_.load()) {
            result.reason = "local_planner_cancelled";
            ::kill(pid, SIGKILL);
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_ms && result.reason.empty()) {
            result.timed_out = true;
            result.reason = "local_planner_timeout";
            ::kill(pid, SIGKILL);
        }

        pollfd descriptor{};
        descriptor.fd = fd;
        descriptor.events = POLLIN | POLLHUP;
        const int poll_result = ::poll(&descriptor, 1, 50);
        if (poll_result > 0 &&
            (descriptor.revents & (POLLIN | POLLHUP)) != 0) {
            char buffer[4096];
            const ssize_t count = ::recv(fd, buffer, sizeof(buffer), 0);
            if (count > 0) {
                result.output.append(buffer, static_cast<std::size_t>(count));
                if (result.output.size() > max_output && result.reason.empty()) {
                    result.reason = "local_planner_output_too_large";
                    ::kill(pid, SIGKILL);
                }
            }
        }

        const pid_t waited = ::waitpid(pid, &child_status, WNOHANG);
        if (waited == pid) {
            child_exited = true;
        } else if (waited < 0 && errno != EINTR) {
            result.reason =
                std::string("local_planner_waitpid_failed:") +
                std::strerror(errno);
            child_exited = true;
        }
    }

    // Drain the final bytes written immediately before process exit.
    while (result.output.size() <= max_output) {
        char buffer[4096];
        const ssize_t count = ::recv(fd, buffer, sizeof(buffer), MSG_DONTWAIT);
        if (count > 0) {
            result.output.append(buffer, static_cast<std::size_t>(count));
            continue;
        }
        break;
    }
    ::close(fd);

    if (WIFEXITED(child_status)) {
        result.exit_code = WEXITSTATUS(child_status);
    } else if (WIFSIGNALED(child_status)) {
        result.exit_code = 128 + WTERMSIG(child_status);
    }

    if (result.output.size() > max_output) {
        result.output.clear();
        result.reason = "local_planner_output_too_large";
    } else if (result.reason.empty() && result.exit_code != 0) {
        result.reason = "local_planner_process_failed_exit_" +
                        std::to_string(result.exit_code);
    } else if (result.reason.empty()) {
        result.reason = "ok";
    }
    return result;
}

}  // namespace laas
