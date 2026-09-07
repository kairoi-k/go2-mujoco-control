#pragma once
#include "terrain_planner.h"
#include "go2_rigid_body.h"
#include <iomanip>
#include <ostream>
#include <sstream>
namespace go2_trot {
// Exact worker inputs for offline replay. Source/model/binary hashes belong
// to the run manifest. Null preserves unknown numeric map fields; it never
// becomes an invented zero height or measured contact in this serialization.
inline std::string JointShadowSnapshotJson(const go2_control::RigidBodyState &state,
    const go2_terrain::TerrainPlannerInput &input,std::uint64_t id,int pattern) {
 std::ostringstream o;o.precision(17);
 auto number=[&](double x){if(std::isfinite(x))o<<x;else o<<"null";};
 auto vector=[&](const auto &v,int n){o<<'[';for(int j=0;j<n;++j){if(j)o<<',';number(v[j]);}o<<']';};
 auto feet=[&](const auto &f){o<<'[';for(int l=0;l<4;++l){if(l)o<<',';const auto&p=f[l];o<<'[';number(p.x);o<<',';number(p.y);o<<',';number(p.z);o<<']';}o<<']';};
 o<<"{\"schema\":\"joint-shadow-snapshot-v1\",\"id\":"<<id<<",\"time\":";number(input.state_stamp_s);
 o<<",\"pattern\":"<<pattern<<",\"phase\":";number(input.gait_phase);o<<",\"period\":";number(input.gait_period_s);
 o<<",\"duty\":";number(input.duty_factor);o<<",\"command_vx\":";number(input.commanded_vx_mps);
 o<<",\"base_yaw\":";number(input.base_yaw_rad);
 o<<",\"position\":";vector(state.position_world,3);
 const std::array<double,4> quat{{state.quat_world_from_body.w(),state.quat_world_from_body.x(),state.quat_world_from_body.y(),state.quat_world_from_body.z()}};
 o<<",\"quaternion_wxyz\":";vector(quat,4);o<<",\"linear_velocity_world\":";vector(state.linear_vel_world,3);
 o<<",\"angular_velocity_body\":";vector(state.angular_vel_body,3);o<<",\"q\":";vector(state.q,12);o<<",\"dq\":";vector(state.dq,12);
 o<<",\"measured_valid\":"<<(input.contact_schedule.measured_valid?"true":"false")<<",\"measured_contact\":[";
 for(int l=0;l<4;++l){if(l)o<<',';o<<(input.contact_schedule.measured_contact[l]?"true":"false");}o<<']';
 o<<",\"touchdown_reference_valid\":"<<(input.touchdown_target_feet_valid?"true":"false")<<",\"touchdown_reference_feet_base\":";feet(input.touchdown_target_feet_base);
 o<<",\"terrain\":";
 if(!input.terrain)o<<"null";
 else {const auto&m=*input.terrain;o<<"{\"frame\":"<<std::quoted(m.frame_id)<<",\"source\":"<<static_cast<int>(m.source)<<",\"epoch\":"<<m.epoch;
 o<<",\"registered\":"<<(m.registered?"true":"false")<<",\"map_sequence\":"<<m.map_sequence;
 o<<",\"state_stamp\":";number(m.state_stamp_s);o<<",\"map_stamp\":";number(m.map_stamp_s);o<<",\"age\":";number(m.age_s);
 o<<",\"width\":"<<m.width<<",\"height\":"<<m.height<<",\"resolution\":";number(m.resolution_m);o<<",\"origin\":";vector(m.origin_m,2);
 o<<",\"registration_position\":";vector(m.registration_position_world,3);o<<",\"registration_yaw\":";number(m.registration_yaw_rad);
 o<<",\"capture_position\":";vector(m.capture_position_world,3);o<<",\"capture_yaw\":";number(m.capture_yaw_rad);
 o<<",\"cell_fields\":[\"known\",\"height\",\"has_bounds\",\"min\",\"max\",\"age\",\"slope\",\"roughness\",\"variance\",\"nx\",\"ny\",\"nz\"],\"cells\":[";
 for(std::size_t j=0;j<m.cells.size();++j){if(j)o<<',';const auto&c=m.cells[j];o<<'['<<(c.known?"true":"false")<<',';number(c.height_m);o<<','<<(c.has_height_bounds?"true":"false")<<',';
 number(c.height_min_m);o<<',';number(c.height_max_m);o<<',';number(c.age_s);o<<',';number(c.slope_rad);o<<',';number(c.roughness_m);o<<',';number(c.variance_m2);
 for(double n:c.normal){o<<',';number(n);}o<<']';}o<<"]}";}
 o<<'}';return o.str();
}
} // namespace
