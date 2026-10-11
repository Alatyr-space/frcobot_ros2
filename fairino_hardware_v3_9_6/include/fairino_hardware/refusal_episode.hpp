#ifndef FAIRINO_HARDWARE__REFUSAL_EPISODE_HPP_
#define FAIRINO_HARDWARE__REFUSAL_EPISODE_HPP_

// F-226: the controller's answers to ServoJ, one per control cycle, turned
// into the few events worth saying: a refusal episode started, its code
// changed, it ended, or the last word is a second old and is due again.
// Pure: no ROS, no SDK; `now_ns` is write()'s own time.

#include <cstdint>

namespace fairino_hardware
{

class RefusalEpisode
{
public:
  enum class Event {None, Started, CodeChanged, Ended, RepeatDue};

  explicit RefusalEpisode(int64_t repeat_ns = 1000000000)
  : repeat_ns_(repeat_ns) {}

  // One ServoJ answer: 0 is accepted, anything else a refusal.
  Event observe(int code, int64_t now_ns)
  {
    Event event = Event::None;
    if (code != 0) {
      if (!active_) {
        active_ = true;
        code_ = code;
        refusals_ = 0;
        since_ns_ = now_ns;
        event = Event::Started;
      } else if (code != code_) {
        code_ = code;
        event = Event::CodeChanged;
      }
      ++refusals_;
    } else if (active_) {
      active_ = false;
      ended_ns_ = now_ns;
      event = Event::Ended;
    }
    if (event == Event::None && (!said_ || now_ns - said_ns_ >= repeat_ns_)) {
      event = Event::RepeatDue;
    }
    return event;
  }

  // The last word went out at `now_ns` (a publisher may be busy: until this
  // is called, the event stays due).
  void said(int64_t now_ns)
  {
    said_ = true;
    said_ns_ = now_ns;
  }

  // The episode is over, but its word has not gone out.
  void unsaid() {said_ = false;}

  bool active() const {return active_;}
  int code() const {return code_;}
  int refusals() const {return refusals_;}
  int64_t sinceNs() const {return since_ns_;}
  int64_t endedNs() const {return ended_ns_;}

private:
  int64_t repeat_ns_;
  bool active_ = false;
  int code_ = 0;
  int refusals_ = 0;
  int64_t since_ns_ = 0;
  int64_t ended_ns_ = 0;
  bool said_ = false;
  int64_t said_ns_ = 0;
};

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__REFUSAL_EPISODE_HPP_
