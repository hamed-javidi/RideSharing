// Copyright (c) 2019 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <iostream>
#include <algorithm> /* min(), max() */
#include <condition_variable>
#include <fstream>
#include <map>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include "libcargo/file.h"
#include "libcargo/types.h"
#include "libcargo/cargo.h"
#include "libcargo/classes.h"

#include "libcargo.h"
#include "bilateral+/bilateral+.h"
#include "bilateral_arrangement/bilateral_arrangement.h"
#include "grabby/grabby.h"
#include "grasp/grasp.h"
#include "greedy/greedy.h"
#include "kinetic_tree/kinetic_tree.h"
#include "nearest_neighbor/nearest_neighbor.h"
#include "nearest_road/nearest_road.h"
#include "simulated_annealing/simulated_annealing.h"
#include "trip_vehicle_grouping/trip_vehicle_grouping.h"
#include "tshare/tshare.h"
#include "bbo/bbo.h"
#include "pso/pso.h"
void Add_LatestDepartureTime_to_problemFile() {
   long tid;
   long oid;
   long did;
   int q;
   long early;
   long late;
   std::string s1, s2, s3, s4, s5, s6;

   std::ifstream ifs(
         "/home/hamed/git/RideSharing/Cargo_benchmark/problem/rs-bj5non-m1k-c3-d6-s10-x1.0.instance");
   std::ofstream if2(
         "/home/hamed/git/RideSharing/Cargo_benchmark/problem/bj5-m1k-with_LatestDepartureTime.instance");

   std::getline(ifs, s1);
   std::getline(ifs, s2);
   std::getline(ifs, s3);
   std::getline(ifs, s4);
   std::getline(ifs, s5);
   std::getline(ifs, s6);
   if2 << s1 << '\n' << s2 << '\n' << s3 << '\n' << s4 << '\n' << s5 << '\n'
       << s6 << '\t' << "LateDep" << '\n';
   int ldep = 0;
   long distance = 0;
   while (ifs >> tid >> oid >> did >> q >> early >> late) {
      distance = get_shortest_path(oid, did);  // in meter
      ldep = late - distance / 10.0;  // distance in meter / speed in meter per second speed 10 meter per sec means 36 kmps in avg
      if (ldep < 0) {
         late = distance / 10.0 + 1200;  //have 20 min slack
         ldep = 1200;
      }

      if2 << tid << '\t' << oid << '\t' << did << '\t' << q << '\t' << early
          << '\t' << late << '\t' << ldep << '\n';

   }
   ifs.close();
   if2.close();
}
void print_header() {
   std::cout << "-----------------------------------------------------------\n"
             << " C A R G O -- Ridesharing Algorithms Benchmarking Platform \n"
             << "-----------------------------------------------------------"
             << std::endl;
}

void print_usage() {
   std::cout
         << "Interactive:  ./launcher\n"
         << "Command-line: ./launcher selection(1-12) *.rnet *.instance [static(0-1)] [strict(0-1)] \n"
         << std::endl;
}

int main(int argc, char **argv) {
   print_header();
   print_usage();
   Options op;
   std::string selection, roadnetwork, instance;
   bool staticmode = false;
   bool strictmode = false;
   if (argc >= 4 && argc <= 6) {
      vec_t<std::string> args(argv, argv + argc);
      selection = args.at(1);
      roadnetwork = args.at(2);
      instance = args.at(3);
      if (argc >= 5)
         staticmode = (args.at(4) == "0" ? false : true);
      if (argc == 6)
         strictmode = (args.at(5) == "0" ? false : true);
   } else {
      if (argc > 6) {
         std::cout << "Too many arguments!" << std::endl;
         print_usage();
      }
      std::cout
            << "Make a selection.\n"
            << "Algorithms:\n"
            //<< "    1) bilateral+               4) greedy\n"
            //<< "    2) bilateral_arrangement    5) kinetic_tree\n"
            //<< "    3) grabby                   6) nearest_neighbor\n"
            // << "  Join Algorithms:\n"
            //<< "    7) grasp4                  10) sa100\n"
            //<< "    8) grasp16                 11) trip_vehicle_grouping\n"
            //<< "    9) sa50\n" << "  Other:\n"
            //<< "    12) nearest_road			 13)T-share\n"
            << " 4) greedy\n"
            << " 5) kinetic_tree\n" << " 10) Simulated Annealing\n"
            << " 13) T-share\n" << " 14) BBO \n"

            << "Your selection (Enter one of above number): ";

      std::cin >> selection;
      //selection = "14";
      std::cout << "Path to rnet (*.rnet): ";
      //std::cin >> roadnetwork;
      roadnetwork = "Cargo_benchmark/road/bj5.rnet";
      std::cout << roadnetwork;
      std::cout << "Path to instance (*.instance): ";
      //std::cin >> instance;
      instance =
            "Cargo_benchmark/problem/rs-bj5non-m200-c3-d6-s10-x1.0.instance";
      std::cout << instance;
   }

   op.path_to_roadnet = roadnetwork;
   op.path_to_problem = instance;
   op.static_mode = staticmode;
   op.strict_mode = strictmode;
   Cargo cargo(op);
   //Add_LatestDepartureTime_to_problemFile();

   if (selection == "1") {
      BilateralPlus alg("bp_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "2") {
      BilateralArrangement alg("ba_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "3") {
      Grabby alg("gb_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "4") {
      Greedy alg("gr_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "5") {
      KineticTrees alg("kt_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "6") {
      NearestNeighbor alg("nn_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "7") {
      GRASP alg("gp4_" + cargo.name(), 4);
      cargo.start(alg);
   } else if (selection == "8") {
      GRASP alg("gp16_" + cargo.name(), 16);
      cargo.start(alg);
   } else if (selection == "9") {
      SimulatedAnnealing alg("sa50_" + cargo.name(), 50);
      cargo.start(alg);
   } else if (selection == "10") {
      SimulatedAnnealing alg("sa100_" + cargo.name(), 100);
      cargo.start(alg);
   } else if (selection == "11") {
      TripVehicleGrouping alg("tg_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "12") {
      NearestRoad alg("nr_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "13") {
      Tshare alg("ts_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "14") {
      BBO alg("bbo_" + cargo.name());
      cargo.start(alg);
   } else if (selection == "15") {
      PSO alg("pso_" + cargo.name());
      cargo.start(alg);
   }
   //Hamed create new instance file:
   else if (selection == "16") {
      TripId tid;
      OrigId oid;
      DestId did;
      Load q;
      ErlyTime early;
      LateTime late;
      std::string _;
      std::ifstream ifs(
            "/home/hamed/Desktop/Cargo-master/Cargo_benchmark/problem/rs-bj5-m5k-c3-d6-s10-x1.0.instance");
      if (!ifs.good())
         throw std::runtime_error("problem path 1 not found");
      std::ofstream if2(
            "/home/hamed/Desktop/Cargo-master/Cargo_benchmark/problem/Edited_problems/rs-bj5non-m5k-c3-d6-s10-x1.0.instance");
      if (!if2.good())
         throw std::runtime_error("problem path 2 not found");
      ifs >> _ >> _ >> _ >> _ >> _ >> _ >> _;
      ifs >> _;
      std::getline(ifs, _);
      vec_t<long> dest;
      while (ifs >> tid >> oid >> did >> q >> early >> late) {
         if (did == -1)
            dest.push_back(oid);
      }
      sort(dest.begin(), dest.end());
      ifs.close();
      ifs.open(
            "/home/hamed/Desktop/Cargo-master/Cargo_benchmark/problem/rs-bj5-m5k-c3-d6-s10-x1.0.instance");
      std::getline(ifs, _);
      std::getline(ifs, _);
      std::getline(ifs, _);
      std::getline(ifs, _);
      std::getline(ifs, _);
      std::getline(ifs, _);
      while (ifs >> tid >> oid >> did >> q >> early >> late) {
         if (did == -1) {
            if2 << tid << '\t' << oid << '\t' << dest.front() << '\t' << q
                << '\t' << early << '\t' << late << '\n';
            dest.erase(dest.begin());
         } else
            if2 << tid << '\t' << oid << '\t' << did << '\t' << q << '\t'
                << early << '\t' << late << '\n';

      }
      ifs.close();
      if2.close();
      //end Hamed
   }

}

