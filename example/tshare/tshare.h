// Copyright (c) 2018 the Cargo authors
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
#include "libcargo.h"

using namespace cargo;
struct cell{
	int id;
	dict<int, DistInt> spatial; //index , distance in meter
	dict<int, int> temporal; //index , time in minutes
	vec_t<std::pair<int , MutableVehicleSptr> > taxies;  //id and time
};

class Tshare : public RSAlgorithm {
 public:
  Tshare(const std::string &);
  int grid_size;
  Point center[10][10];
  NodeId centernode[10][10];

  /* My overrides */
  virtual void handle_customer(const Customer &);
  virtual void handle_vehicle(const Vehicle &);
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);
  vec_t<MutableVehicleSptr> dualside_matching(const Customer & cust);
  long lazyshortestpath(NodeId orig, NodeId dest);
  bool insertion_check(const Customer& cust, const MutableVehicleSptr& V);
  int node2gridcell(const NodeId node);
  vec_t<MutableVehicleSptr>  Check_pickup_time_window(int ,int, LateTime );
  vec_t<MutableVehicleSptr>  Check_dropoff_time_window(int , LateTime );
  void clear_gridcell_taxies();
  void insert_gridcell_taxies(const Vehicle& vehl);



  std::vector <std::vector <int> > distance_matrix;
  vec_t<vec_t<cell>> gridcell;
  //vector <Player> players;
  NodeId pt2node(Point& i);
  void fun1();
 private:
  bool sort_now;
  Grid grid_;
  int error_;
  /* Workspace variables */
  DistInt best_cost;
  vec_t<Stop> sch, best_sch;
  vec_t<Wayp> rte, best_rte;
  MutableVehicleSptr best_vehl;
  vec_t<MutableVehicleSptr> candidates;
  bool matched;
  tick_t timeout_0;
  vec_t<long> vlate;
  vec_t<long> vearly;
  void reset_workspace();
};
vec_t<MutableVehicleSptr> set_intersection_MutableVehicleSptr(vec_t<MutableVehicleSptr> a, vec_t<MutableVehicleSptr> b);
bool sortfunction (std::pair<int , MutableVehicleSptr> i,std::pair<int , MutableVehicleSptr> j) ;


