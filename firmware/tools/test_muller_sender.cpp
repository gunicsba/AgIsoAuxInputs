// PC test of src/muller_sender.h (from firmware/):
//   g++ -std=c++17 -Wall -Wextra -o test_muller_sender tools/test_muller_sender.cpp && ./test_muller_sender
#include "../src/muller_sender.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

#define CHECK(condition)                                                    \
  do {                                                                      \
    if (!(condition)) {                                                     \
      printf("FAILED line %d: %s\n", __LINE__, #condition);                 \
      exit(1);                                                              \
    }                                                                       \
  } while (0)

struct Frame {
  uint32_t at;
  uint8_t nn, cc, ss;
};

// Runs the sender as loop() does: update every 5 ms, buttons polled every 40.
struct Rig {
  MullerSender sender;
  uint8_t map[16];
  uint32_t now = 0xFFFFF000; // runs through the millis() wrap
  uint32_t start = now;
  uint16_t buttons = 0;
  bool ready = true, queueFull = false;
  std::vector<Frame> sent;

  Rig() {
    for (auto &m : map)
      m = MULLER_NO_FUNCTION;
  }
  void run(uint32_t ms) {
    for (uint32_t end = now + ms; now != end; now += 5) {
      if ((now - start) % 40 == 0)
        sender.report(buttons, map);
      sender.update(now, ready, [&](const uint8_t *f) {
        if (queueFull)
          return false;
        CHECK(f[0] == 0x21 && f[2] == 0xFF && f[3] == 0xFF && f[5] == 0 && f[7] == 0xFF);
        sent.push_back({now - start, f[1], f[4], f[6]});
        return true;
      });
    }
  }
  std::vector<Frame> take() {
    auto s = sent;
    sent.clear();
    return s;
  }
};

int main() {
  { // press, hold, release; the counter goes up once per press
    Rig r;
    r.map[3] = 18;
    r.run(400);
    CHECK(r.take().empty());
    for (int press = 1; press <= 300; press++) {
      r.buttons = 1 << 3;
      r.run(1000);
      r.buttons = 0;
      r.run(200);
      auto f = r.take();
      CHECK(f.size() == 6);
      for (size_t i = 0; i < f.size(); i++) {
        CHECK(f[i].nn == 18 && f[i].cc == (uint8_t)press);
        CHECK(f[i].ss == (i == 0 ? 1 : i == f.size() - 1 ? 0 : 2));
        if (i > 0)
          CHECK(f[i].at - f[i - 1].at == 200);
      }
    }
  }
  { // unmapped buttons send nothing
    Rig r;
    r.buttons = 0xFFFF;
    r.run(1000);
    CHECK(r.take().empty());
  }
  { // two buttons on one function: one press, released with the last one
    Rig r;
    r.map[1] = r.map[2] = 9;
    r.buttons = 2;
    r.run(80);
    r.buttons = 6;
    r.run(80);
    r.buttons = 4;
    r.run(80);
    r.buttons = 0;
    r.run(80);
    auto f = r.take();
    CHECK(f.size() == 3 && f[0].ss == 1 && f[1].ss == 2 && f[2].ss == 0);
  }
  { // a mapping change while held doesn't move or lose the release
    Rig r;
    r.map[1] = 20;
    r.buttons = 2;
    r.run(400);
    r.map[1] = 21;
    r.run(400);
    r.buttons = 0;
    r.run(80);
    auto f = r.take();
    CHECK(!f.empty() && f.back().ss == 0);
    for (auto &x : f)
      CHECK(x.nn == 20);
  }
  { // bus lost and back within 300 ms or later: release, no restart while held
    for (uint32_t gap : {20u, 100u, 5000u}) {
      Rig r;
      r.map[1] = 20;
      r.buttons = 2;
      r.run(400);
      r.take();
      r.ready = false;
      r.run(gap);
      CHECK(r.take().empty());
      r.ready = true;
      r.run(2000);
      auto f = r.take();
      CHECK(f.size() == 1 && f[0].ss == 0);
      r.buttons = 0;
      r.run(2000 - gap % 40);
      r.buttons = 2;
      r.run(80);
      f = r.take();
      CHECK(f.size() == 1 && f[0].ss == 1 && f[0].cc == 2);
    }
  }
  { // pressed before the address claim: nothing until pressed again
    Rig r;
    r.map[1] = 20;
    r.ready = false;
    r.buttons = 2;
    r.run(1000);
    r.ready = true;
    r.run(1000);
    CHECK(r.take().empty());
  }
  { // released while the bus was down: release sent afterwards
    Rig r;
    r.map[1] = 20;
    r.buttons = 2;
    r.run(80);
    r.ready = false;
    r.run(80);
    r.buttons = 0;
    r.run(80);
    r.ready = true;
    r.run(80);
    auto f = r.take();
    CHECK(f.size() == 2 && f[0].ss == 1 && f[1].ss == 0);
  }
  { // loop stalled past 300 ms: release, no restart while held
    Rig r;
    r.map[1] = 20;
    r.buttons = 2;
    r.run(80);
    r.take();
    r.now += 800;
    r.run(1000);
    auto f = r.take();
    CHECK(f.size() == 1 && f[0].ss == 0);
  }
  { // TX queue full at the release: retried
    Rig r;
    r.map[1] = 20;
    r.buttons = 2;
    r.run(80);
    r.queueFull = true;
    r.buttons = 0;
    r.run(80);
    r.queueFull = false;
    r.run(5);
    auto f = r.take();
    CHECK(f.size() == 2 && f[1].ss == 0);
  }
  printf("OK\n");
}
