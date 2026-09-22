// Cross-engine virtual-work check. Server compensation must match physical
// body potential for every leg, arbitrary joint posture and chassis attitude.
#include "minphys3d/demo/pinocchio_hexapod.hpp"
#include "control/joint_angle_gravity_feedforward.hpp"
#include "hardware/physics_sim_joint_wire_mapping.hpp"
#include "hexapod_dynamics_constants.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace mp = minphys3d;
namespace demo = minphys3d::demo;
int main() {
    mp::World world({0,-9.80665,0});
    const auto scene = demo::BuildHexapodScene(world);
    std::array<std::uint32_t,18> ids{};
    std::array<LegGeometry,6> legs{};
    for (int l=0;l<6;++l) {
        ids[3*l]=scene.legs[l].bodyToCoxaJoint;
        ids[3*l+1]=scene.legs[l].coxaToFemurJoint;
        ids[3*l+2]=scene.legs[l].femurToTibiaJoint;
        const auto axis = mp::Rotate(world.GetBody(scene.legs[l].coxa).orientation, mp::Vec3{1,0,0});
        legs[l].mountAngle=AngleRad{kPi-std::atan2(axis.x,-axis.z)};
        legs[l].coxaLength=LengthM{.043}; legs[l].femurLength=LengthM{.060}; legs[l].tibiaLength=LengthM{.104};
    }
    demo::PinocchioHexapodModel model(world,scene,ids);
    std::vector<double> q,v;
    model.readState(world,q,v);
    const auto initial=q;
    std::fill(v.begin(),v.end(),0);
    auto potential=[&]() {
        double u=0;
        auto add=[&](std::uint32_t id) { const auto& b=world.GetBody(id); u-=b.mass*mp::Dot(world.GetGravity(),b.position); };
        add(scene.body);
        for (const auto& l:scene.legs) { add(l.coxa);add(l.femur);add(l.tibia); }
        return u;
    };
    control_config::GravityFeedforwardConfig cfg{};
    cfg.include_self_weight=true; cfg.include_foot_reaction=false;
    double worst=0;
    for (int seed=0;seed<250;++seed) {
        q=initial;
        q[3]=.2*std::sin(seed*.31); q[4]=.3*std::cos(seed*.47); q[5]=.5*std::sin(seed*.23);q[6]=1;
        const double n=std::sqrt(q[3]*q[3]+q[4]*q[4]+q[5]*q[5]+1);
        for (int k=3;k<7;++k) q[k]/=n;
        for (int j=0;j<18;++j) q[7+j]+=.5*std::sin(seed*.13+j*.7);
        model.writeState(world,q,v);
        const auto g=mp::Rotate(mp::Conjugate(world.GetBody(scene.body).orientation),mp::Vec3{0,-1,0});
        const Vec3 down{-g.z,g.x,g.y};
        for (int l=0;l<6;++l) {
            std::array<double,3> mechanical{};
            for (int j=0;j<3;++j)
                mechanical[j]=physics_sim_joint_wire_mapping::jointMechanicalFromSimWireAngle(j,world.GetServoJointAngle(ids[3*l+j]));
            const auto predicted=computeLegGravityCompensation(legs[l],mechanical[0],mechanical[1],mechanical[2],down,0,cfg);
            for (int j=1;j<3;++j) {
                auto perturbed=q; perturbed[7+3*l+j]+=1e-6; model.writeState(world,perturbed,v);
                const double plus=potential(); perturbed[7+3*l+j]-=2e-6; model.writeState(world,perturbed,v);
                const double gradient=(plus-potential())/2e-6;
                const double predictedTorque=j==1?predicted.torque_femur_nm:predicted.torque_tibia_nm;
                worst=std::max(worst,std::abs(gradient-predictedTorque));
                if (std::abs(gradient-predictedTorque)>1e-7) {
                    std::cerr<<"gravity mismatch seed="<<seed<<" leg="<<l<<" joint="<<j
                             <<" predicted="<<predictedTorque<<" physical="<<gradient<<'\n'; return 1;
                }
                model.writeState(world,q,v);
            }
        }
    }
    std::cout<<"server gravity agrees with physical potential: poses=250 legs=6 pitch_checks=3000 max_error_nm="<<worst<<'\n';
}
