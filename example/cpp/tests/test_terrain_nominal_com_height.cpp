#include "terrain_nominal_com_height.h"
#include <cstdlib>
#include <iostream>
int main() {
    using go2_trot::MakeNominalComHeight;
    int checks=0;
    auto check=[&](bool ok){++checks;if(!ok){std::cerr<<"failed "<<checks<<"\n";std::exit(1);}};
    const std::array<double,4> neutral{-.35,-.35,-.35,-.35};
    const std::array<bool,4> all{true,true,true,true};
    auto r=MakeNominalComHeight({.022,.022,.022,.022},neutral,all,-.027);
    check(r.valid && std::abs(r.base_world_z-.372)<1e-12 && std::abs(r.com_world_z-.345)<1e-12);
    r=MakeNominalComHeight({.072,.072,.072,.072},neutral,all,-.027);
    check(r.valid && std::abs(r.com_world_z-.395)<1e-12);
    r=MakeNominalComHeight({.072,.072,.022,.022},neutral,all,-.027);
    check(r.valid && std::abs(r.com_world_z-.345)<1e-12);
    r=MakeNominalComHeight({10.022,10.022,10.022,10.022},neutral,all,-.027);
    check(r.valid && std::abs(r.com_world_z-10.345)<1e-12);
    check(!MakeNominalComHeight({},neutral,{},-.027).valid);
    check(!MakeNominalComHeight({NAN,.022,.022,.022},neutral,all,-.027).valid);
    check(!MakeNominalComHeight({.022,.022,.022,.022},neutral,all,NAN).valid);
    check(!MakeNominalComHeight({.022,.022,.022,.022},{.35,-.35,-.35,-.35},all,-.027).valid);
    r=MakeNominalComHeight({NAN,.022,.022,.022},neutral,{false,true,true,true},-.027);
    check(r.valid && std::abs(r.com_world_z-.345)<1e-12);
    // A body-height constant used as COM height misses the COM/base offset
    // and neutral foot geometry. This witness intentionally distinguishes it.
    check(std::abs(r.com_world_z-.42)>.07);
    std::cout<<checks<<" nominal COM reference checks PASS\n";
}
