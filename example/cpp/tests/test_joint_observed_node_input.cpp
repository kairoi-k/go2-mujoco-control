#include "../tools/research/joint_observed_node_input.h"
#include <iostream>
namespace input=joint_observed_node;
std::string Model(bool unknown=false,double height=0){std::ostringstream o;o<<"base_link 1 1 1 10 1 1 0 1 1 1 -.5 -.5 0 0 0 0 0 0 0 0 ";if(unknown)o<<"0 nan 0 nan nan inf inf inf inf 0 0 1";else o<<"1 "<<height<<" 1 "<<height<<' '<<height<<" 0 0 0 0 0 0 1";return o.str();}
std::string Fixture(bool unknown=false){std::ostringstream o;o<<"joint-shadow-snapshot-v2 1 1 1 .2 .14 .44 .9 0 0 0 .3 1 0 0 0 ";for(int i=0;i<30;++i)o<<"0 ";o<<"1 1 0 0 1 0 ";for(int i=0;i<12;++i)o<<"0 ";o<<Model(unknown)<<" 1 1 1 .001 .9998 1 "<<Model(unknown);return o.str();}
int main(){int checks=0;auto require=[&](bool ok){if(!ok)throw std::runtime_error("check failed");++checks;};auto reject=[&](std::string text){bool rejected=false;try{std::istringstream stream(text);input::Parse(stream);}catch(const std::exception&){rejected=true;}require(rejected);};try{
 auto text=Fixture();std::istringstream stream(text);auto parsed=input::Parse(stream);require(parsed.tokens>46&&parsed.history.captures.size()==1&&parsed.latest.cells.size()==1&&parsed.measured_contact[0]);
 reject(text+" extra");reject(text.substr(0,text.find_last_of(' ')));reject(text.substr(0,100));
 auto mismatch=text;auto pos=mismatch.find(Model());mismatch.replace(pos,Model().size(),Model(false,.1));reject(mismatch);
 auto badtime=text;pos=badtime.find(" 1 1 1 .001 .9998 1 ");badtime.replace(pos,19," 1 1.1 1 .001 .9998 1 ");reject(badtime);
 std::istringstream unknown(Fixture(true));auto unavailable=input::Parse(unknown);require(!unavailable.latest.cells[0].known&&std::isnan(unavailable.latest.cells[0].height_m));auto query=go2_terrain::stage_c::SampleWorldTerrainSnapshot(unavailable.history,0,0,0,.1);require(!query.ok());
 reject("joint-shadow-snapshot-v1"+text.substr(text.find(' ')));auto malformed=text;pos=malformed.find("base_link");malformed.replace(pos,Model().size(),"base_link 1 1 1 10 1 1 0 4294967297 1 1");reject(malformed);
 std::cout<<"checks_passed="<<checks<<"\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
