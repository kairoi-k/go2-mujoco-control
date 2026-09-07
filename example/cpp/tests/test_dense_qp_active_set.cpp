#include "dense_qp_active_set.h"
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
using namespace go2_control;
void Check(bool v,const char*s){if(!v)throw std::runtime_error(s);}
int main(int argc,char**argv){try{
 Eigen::MatrixXd H=Eigen::MatrixXd::Identity(2,2),A(4,2),E(0,2);
 Eigen::VectorXd g(2),b(4),d(0),seed=Eigen::VectorXd::Zero(2),x;
 g<<-2,-3;A<<1,0,0,1,-1,0,0,-1;b<<1,2,0,0;int iterations=0;
 Check(SolveDenseQpPrimalActiveSet(H,g,A,b,E,d,seed,x,iterations),"box solve");
 Check((x-Eigen::Vector2d(1,2)).norm()<1e-9,"analytic box optimum");
 A.conservativeResize(5,2);b.conservativeResize(5);A.row(4)=2*A.row(0);b[4]=2;
 Check(SolveDenseQpPrimalActiveSet(H,g,A,b,E,d,seed,x,iterations),"redundant inequalities");
 Check((x-Eigen::Vector2d(1,2)).norm()<1e-9,"redundant optimum");
 E.resize(1,2);E<<1,1;d.resize(1);d<<1;seed<<.5,.5;
 Check(SolveDenseQpPrimalActiveSet(H,g,A,b,E,d,seed,x,iterations),"equality solve");
 Check((x-Eigen::Vector2d(0,1)).norm()<1e-8,"analytic equality optimum");
 seed<<5,5;Check(!SolveDenseQpPrimalActiveSet(H,g,A,b,E,d,seed,x,iterations),"invalid seed rejected");
 seed<<.5,.5;H=-H;
 Check(!SolveDenseQpPrimalActiveSet(H,g,A,b,E,d,seed,x,iterations),"nonconvex reduced Hessian rejected");
 if(argc>1){
  std::ifstream f(argv[1]);Check(bool(f),"fixture open");std::map<std::string,Eigen::MatrixXd> q;std::string name;int rows,cols;
  while(f>>name>>rows>>cols){Eigen::MatrixXd m(rows,cols);for(int r=0;r<rows;++r)for(int c=0;c<cols;++c)Check(bool(f>>m(r,c)),"matrix parse");q[name]=m;}
  H=q.at("H");g=q.at("g");A=q.at("Aineq");b=q.at("bineq");E=q.at("Aeq");d=q.at("beq");
  // Independent HiGHS witness retained alongside original runtime QP.
  const bool embedded_seed=q.count("seed")!=0;
  if(embedded_seed) seed=q.at("seed");
  else {std::ifstream sf(std::string(argv[1])+".seed");seed.resize(H.rows());for(int j=0;j<seed.size();++j)Check(bool(sf>>seed[j]),"seed parse");}
  Check(SolveDenseQpPrimalActiveSet(H,g,A,b,E,d,seed,x,iterations),"actual WBC matrix");
  const double objective=.5*x.dot(H*x)+g.dot(x);
  Check((E*x-d).lpNorm<Eigen::Infinity>()<1e-7,"actual equality certificate");
  Check((A*x-b).maxCoeff()<1e-7,"actual inequality certificate");
  Check(std::abs(objective-(embedded_seed ? 17524222.67510441 : -97581902.95771791))<.01,"independent SLSQP optimum");
  const auto lp_seed_optimum=x;
  if(!embedded_seed) {
  std::ifstream ff(std::string(argv[1])+".freefall_seed");
  for(int j=0;j<seed.size();++j)Check(bool(ff>>seed[j]),"freefall seed parse");
  Check(SolveDenseQpPrimalActiveSet(H,g,A,b,E,d,seed,x,iterations),"actual freefall seed solve");
  Check((x-lp_seed_optimum).norm()<1e-5,"seed independent optimum");
  }
  const auto first=x;const int first_iterations=iterations;
  Check(SolveDenseQpPrimalActiveSet(H,g,A,b,E,d,seed,x,iterations),"deterministic repeat");
  Check((x-first).norm()==0 && first_iterations==iterations,"deterministic exact repeat");
  std::cout.precision(17);std::cout<<"actual objective="<<objective<<" iterations="<<iterations<<" eq="<<(E*x-d).lpNorm<Eigen::Infinity>()<<" ineq="<<(A*x-b).maxCoeff()<<"\n";
 }
 std::cout<<"active set analytic oracles passed\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
