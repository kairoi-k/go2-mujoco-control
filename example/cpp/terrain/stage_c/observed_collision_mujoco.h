#pragma once
// Worker-local research model. Reconstructed terrain is not a surface certificate.
// robot_xml is a trusted caller input: RobotOnly validates topology, not its
// recursive source identity. Caller must independently bind XML/asset hashes
// before compiling; this API does not certify arbitrary same-shaped robot files.
#include "observed_collision_model.h"
#include <mujoco/mujoco.h>
#include <memory>
#include <stdexcept>
namespace go2_terrain { namespace stage_c { namespace observed_collision {
struct ModelDeleter { void operator()(const mjModel* m)const { mj_deleteModel(const_cast<mjModel*>(m)); } };
class MujocoModel;
std::shared_ptr<const MujocoModel> CompileMujoco(std::shared_ptr<const Descriptor>,const std::string&);
class MujocoModel final {
    const std::shared_ptr<const Descriptor> descriptor_;
    const std::unique_ptr<const mjModel,ModelDeleter> model_;
    MujocoModel(std::shared_ptr<const Descriptor> d,mjModel* m):descriptor_(std::move(d)),model_(m){}
    friend std::shared_ptr<const MujocoModel> CompileMujoco(std::shared_ptr<const Descriptor>,const std::string&);
public:
    static constexpr bool can_actuate=false;
    static constexpr bool continuous_surface_certified=false;
    const mjModel* model()const{return model_.get();}
    const Descriptor& descriptor()const{return *descriptor_;}
};
inline bool RobotOnly(const mjModel* m) {
    if(!m||m->nq!=19||m->nv!=18||m->nu!=12||m->njnt!=13||m->na!=0||
       m->nplugin!=0||m->jnt_type[0]!=mjJNT_FREE||m->opt.timestep!=.002)return false;
    const int root=mj_name2id(m,mjOBJ_BODY,"base_link");
    if(root<=0||m->jnt_bodyid[0]!=root||m->body_parentid[root]!=0)return false;
    std::vector<bool> robot(m->nbody,false);robot[root]=true;
    for(int i=root+1;i<m->nbody;++i)robot[i]=robot[m->body_parentid[i]];
    for(int i=1;i<m->nbody;++i)if(!robot[i])return false;
    // Reject imported world geometry outright, including invisible scene terrain.
    for(int g=0;g<m->ngeom;++g)if(!robot[m->geom_bodyid[g]])return false;
    return true;
}
inline std::shared_ptr<const MujocoModel> CompileMujoco(
    std::shared_ptr<const Descriptor> descriptor,const std::string& robot_xml) {
    if(!descriptor||descriptor->prisms().empty())throw std::invalid_argument("missing observed descriptor");
    char error[2048]{};
    std::unique_ptr<mjSpec,decltype(&mj_deleteSpec)> spec(mj_parseXML(robot_xml.c_str(),nullptr,error,sizeof(error)),mj_deleteSpec);
    if(!spec)throw std::runtime_error(error);
    std::unique_ptr<mjModel,ModelDeleter> robot(mj_compile(spec.get(),nullptr));
    if(!robot)throw std::runtime_error(mjs_getError(spec.get()));
    if(!RobotOnly(robot.get()))throw std::invalid_argument("requires robot-only Go2 model without scene geometry");
    auto base=mjs_findBody(spec.get(),"base_link");
    auto world=base?mjs_getParent(base->element):nullptr;
    if(!world)throw std::runtime_error("missing robot world parent");
    for(const auto& cell:descriptor->prisms()) {
        auto geom=mjs_addGeom(world,nullptr);
        if(!geom)throw std::runtime_error("cannot add observed cell");
        const auto name="observed_cell_"+std::to_string(cell.ix)+"_"+std::to_string(cell.iy);
        if(mjs_setName(geom->element,name.c_str()))throw std::runtime_error("cannot name observed cell");
        geom->type=mjGEOM_BOX;
        for(int i=0;i<3;++i){geom->pos[i]=cell.center_world[i];geom->size[i]=cell.half_size[i];}
        geom->quat[0]=std::cos(cell.yaw_rad*.5);geom->quat[1]=0;geom->quat[2]=0;geom->quat[3]=std::sin(cell.yaw_rad*.5);
        // Explicit MuJoCo ground defaults; robot's higher-priority foot settings
        // remain untouched. These are modeling parameters, not sensed friction.
        geom->contype=1;geom->conaffinity=1;geom->condim=3;geom->priority=0;
        geom->friction[0]=1.;geom->friction[1]=.005;geom->friction[2]=.0001;
        geom->margin=0.;geom->gap=0.;
    }
    std::unique_ptr<mjModel,ModelDeleter> compiled(mj_compile(spec.get(),nullptr));
    if(!compiled)throw std::runtime_error(mjs_getError(spec.get()));
    if(compiled->nq!=robot->nq||compiled->nv!=robot->nv||compiled->nu!=robot->nu||
       compiled->nbody!=robot->nbody||compiled->ngeom!=robot->ngeom+static_cast<int>(descriptor->prisms().size()))
        throw std::runtime_error("observed geometry changed robot topology");
    return std::shared_ptr<const MujocoModel>(new const MujocoModel(std::move(descriptor),compiled.release()));
}
}}}
