#include "swing_clearance_census.hpp"
#include <cmath>
#include <iostream>

int main() {
    swing_census::Census c;
    swing_census::Sample s;
    s.contact = true;
    s.body_z = .14; s.foot_body = {0.1, 0, -.14}; s.command_body = s.foot_body;
    c.update(0, s);
    s.stance = false; s.time_s = .005; c.update(0, s);
    s.time_s = .010; s.contact = false;
    // Body falls 20 mm while the joint-relative foot rises 30 mm: net 10 mm.
    s.body_z = .12; s.foot_body.z = -.11; s.command_body.z = -.10;
    s.foot_world_z = .01; c.update(0, s);
    s.time_s = .015; s.contact = true; c.update(0, s);
    s.time_s = .020; s.contact = false; c.update(0, s);
    s.time_s = .025; s.stance = true; c.update(0, s);
    const auto& e = c.events().at(0);
    if (std::abs(e.liftoff_ms - 5) > 1e-8 || e.recontacts != 1 || !e.complete
        || std::abs(e.measured_peak_m - .01) > 1e-8 || e.peak_closure_m > 1e-8
        || std::abs(e.at_command_peak.body + .02) > 1e-8
        || std::abs(e.at_command_peak.joint - .03) > 1e-8) return 1;
    // Never lifted, and interrupted swing: distinguish completed failure/censoring.
    s.time_s = .030; s.stance = false; s.contact = true; c.update(0, s);
    s.time_s = .035; s.stance = true; c.update(0, s);
    s.time_s = .040; s.stance = false; c.update(0, s);
    c.finish(.045);
    if (c.events()[1].liftoff_ms != -1 || !c.events()[1].complete
        || c.events()[2].complete) return 2;
    s.time_s = .05; c.update(1, s); c.finish(.055);
    if (!c.events().back().left_censored) return 3;
    // Rotation contribution and the interaction term must close exactly.
    const auto a = s;
    s.rotation_z = {-.2, 0, std::sqrt(.96)};
    s.foot_body.x += .02;
    s.foot_world_z = s.body_z + swing_census::dot(s.rotation_z, s.foot_body);
    const auto b = swing_census::budget(a, s);
    if (std::abs(b.closure) > 1e-8) return 4;
    // An invalid bus/mode sample must not bridge two unrelated swing histories.
    swing_census::Census interrupted;
    s.stance = true; interrupted.update(0, s);
    s.stance = false; s.time_s += .005; interrupted.update(0, s);
    s.time_s += .005; interrupted.update(0, s, false);
    s.time_s += .005; interrupted.update(0, s);
    interrupted.finish(s.time_s + .005);
    if (interrupted.events().size() != 2 || interrupted.events()[0].complete
        || !interrupted.events()[1].left_censored) return 5;
    std::cout << "swing census event timing, recontact, censoring and geometry passed\n";
}
