#include "../trot/joint_shadow_snapshot.h"
#include <iostream>
int main(int argc,char**){
 go2_control::RigidBodyState s;s.position_world<<1.25,-.3,.41;
 s.quat_world_from_body=Eigen::Quaterniond(Eigen::AngleAxisd(.37,Eigen::Vector3d::UnitZ()));
 s.linear_vel_world<<.9,-.03,.1;s.angular_vel_body<<.05,-.11,.02;
 for(int i=0;i<12;++i){s.q[i]=.1*(i-5);s.dq[i]=.7*(i-4);}
 go2_terrain::TerrainPlannerInput in;in.state_stamp_s=20.25;in.gait_phase=.3;in.gait_period_s=.14;in.duty_factor=.42;in.commanded_vx_mps=.94;in.base_yaw_rad=.37;
 in.contact_schedule.measured_valid=true;in.contact_schedule.measured_contact={true,false,false,true};
 in.touchdown_target_feet_valid=true;for(int i=0;i<4;++i)in.touchdown_target_feet_base[i]={.12*i,-.07*i,-.3};
 go2_terrain::TerrainModel m;m.frame_id="base_link";m.source=go2_terrain::TerrainSource::kTestFixture;m.epoch=7;m.registered=true;m.map_sequence=42;
 m.state_stamp_s=in.state_stamp_s;m.map_stamp_s=20.2;m.age_s=.05;m.width=3;m.height=2;m.resolution_m=.05;m.origin_m={-.1,-.05};
 m.registration_position_world={1.23,-.28,.4};m.registration_yaw_rad=.36;m.capture_position_world={1.21,-.29,.39};m.capture_yaw_rad=.35;m.cells.resize(6);
 for(int i=0;i<5;++i){auto&c=m.cells[i];c.known=true;c.height_m=-.4+.01*i;c.has_height_bounds=true;c.height_min_m=c.height_m-.001;c.height_max_m=c.height_m+.002;c.age_s=.05+.001*i;c.slope_rad=.01;c.roughness_m=.002;c.variance_m2=.000003;c.normal={0,0,1};}
 go2_terrain::stage_c::WorldTerrainSnapshot history;
 if(argc>1){
  m.registration_position_world=m.capture_position_world;m.registration_yaw_rad=m.capture_yaw_rad;
  history.valid=true;history.metadata.aggregate_epoch=m.epoch;history.metadata.state_time_s=in.state_stamp_s;
  history.metadata.stationary_terrain_assumption=true;
  go2_terrain::stage_c::WorldTerrainSnapshotEntry latest;latest.model=m;history.captures.push_back(latest);
  auto older=latest;older.model.map_sequence=41;older.model.map_stamp_s=20.15;older.model.age_s=.1;
  older.model.capture_position_world[0]-=.05;older.model.registration_position_world=older.model.capture_position_world;
  for(auto&c:older.model.cells)if(c.known)c.age_s+=.05;
  history.captures.push_back(older);
 }
 in.terrain=&m;std::cout<<"JointSnapshot "<<go2_trot::JointShadowSnapshotJson(s,in,137,1,argc>1?&history:nullptr)<<"\n";
}
