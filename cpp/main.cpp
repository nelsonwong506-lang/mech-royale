#include "brain.hpp"

#include <cerrno>
#include <iostream>
#include <memory>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>
#include <utility>

namespace mr {
namespace {

void send(const Json& message) {
    std::cout << message.dump() << '\n' << std::flush;
}

Json defaultAction() {
    return Json{
        {"move", {0.0, 0.0}},
        {"aim", {0.0, 0.0}},
        {"boost", false},
        {"jump", false},
        {"fire", {false, false}},
        {"reload", false},
    };
}

class Inbox {
public:
    bool readBatch() {
        if (!readSome()) {
            return false;
        }
        for (;;) {
            pollfd descriptor{STDIN_FILENO, POLLIN, 0};
            const int ready = ::poll(&descriptor, 1, 0);
            if (ready < 0 && errno == EINTR) {
                continue;
            }
            if (ready < 0) {
                throw std::system_error(errno, std::generic_category(), "poll stdin");
            }
            if (ready == 0 || !(descriptor.revents & (POLLIN | POLLHUP)) || !readSome()) {
                return true;
            }
        }
    }

    template <typename Handler>
    bool consume(Handler handle) {
        std::size_t start = 0;
        for (std::size_t end; (end = bytes_.find('\n', start)) != std::string::npos; start = end + 1) {
            auto message = Json::parse(bytes_.begin() + start, bytes_.begin() + end, nullptr, false);
            if (message.is_object() && !handle(std::move(message))) {
                return false;
            }
        }
        bytes_.erase(0, start);
        return true;
    }

private:
    bool readSome() {
        char block[32768];
        ssize_t size;
        do {
            size = ::read(STDIN_FILENO, block, sizeof(block));
        } while (size < 0 && errno == EINTR);
        if (size < 0) {
            throw std::system_error(errno, std::generic_category(), "read stdin");
        }
        bytes_.append(block, static_cast<std::size_t>(size));
        return size != 0;
    }

    std::string bytes_;
};

int run() {
    Inbox inbox;
    std::unique_ptr<Brain> bot;
    Json lastMove = defaultAction();
    while (inbox.readBatch()) {
        std::optional<Json> latest;
        if (!inbox.consume([&](Json message) {
            const auto type = message.value("type", std::string{});
            if (type == "end") {
                return false;
            }
            if (type == "hello") {
                bot = std::make_unique<Brain>(message);
                lastMove = defaultAction();
                send({{"type", "ready"}, {"name", "osu-apex"}});
            } else if (type == "obs") {
                latest = std::move(message);
            }
            return true;
        })) {
            return 0;
        }
        if (!bot || !latest) {
            continue;
        }
        Json action = defaultAction();
        try {
            action = bot->act(*latest);
            lastMove = defaultAction();
            for (const auto* field : {"move", "aim", "boost"}) {
                if (action.contains(field)) {
                    lastMove[field] = action[field];
                }
            }
        } catch (const std::exception& error) {
            std::cerr << "Decision failed: " << error.what() << '\n';
            action = lastMove;
        }
        if (const auto tickIt = latest->find("tick"); tickIt != latest->end()) {
            action["tick"] = *tickIt;
        } else {
            action["tick"] = 0;
        }
        send(action);
    }
    return 0;
}

}  // namespace
}  // namespace mr

int main() {
    std::ios::sync_with_stdio(false);
    try {
        return mr::run();
    } catch (const std::exception& error) {
        std::cerr << "Bot stopped: " << error.what() << '\n';
        return 1;
    }
}
