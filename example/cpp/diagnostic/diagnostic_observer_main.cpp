#include "dds_capture.h"
#include "diagnostic_writer.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
int main(int argc,char**argv){auto d=argc>1?unsigned(std::strtoul(argv[1],nullptr,10)):0u;auto p=argc>2?argv[2]:"diagnostic_observer.jsonl";go2_diagnostic::DdsCapture c(d);if(!c.Start())return 2;std::this_thread::sleep_for(std::chrono::milliseconds(argc>3?std::strtol(argv[3],nullptr,10):1000));c.Stop();go2_diagnostic::DiagnosticWriter w(p);for(auto&r:c.records().captures())w.WriteCapture(*r);for(auto&r:c.records().maps())w.WriteMap(*r);for(auto&r:c.records().states())w.WriteState(*r);std::cout<<"captures="<<c.capture_count()<<std::endl;return 0;}
