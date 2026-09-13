#pragma once
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <stdexcept>
#include <string>

namespace u_robot_a2_driver {
// The logs directory is bind mounted into unitree-dev. Both native navigation
// and the teleop supervisor must hold this lock before initializing SportClient.
class SportControlLock {
public:
  explicit SportControlLock(
    const std::string & path = "/home/unitree/data/logs/a2_sport_control.lock",
    const std::string & owner = "navigation")
  {
    fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd_ < 0) throw std::runtime_error("Cannot open Sport control lock: " + path);
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("Sport control is owned by another navigation/teleop process");
    }
    const auto text = owner + " pid=" + std::to_string(::getpid()) + "\n";
    if (::ftruncate(fd_, 0) != 0 ||
      ::write(fd_, text.data(), text.size()) != static_cast<ssize_t>(text.size()))
    {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("Cannot write Sport control ownership");
    }
  }
  ~SportControlLock() {if (fd_ >= 0) ::close(fd_);}
  SportControlLock(const SportControlLock &) = delete;
  SportControlLock & operator=(const SportControlLock &) = delete;
private:
  int fd_{-1};
};
}  // namespace u_robot_a2_driver
