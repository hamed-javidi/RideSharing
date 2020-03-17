#include "libcargo.h"

using namespace cargo;
//
//struct Driver{
//			int capacity;		// capacity = real capacity - already fixed rides
//			vec_t<int> rider;  //temporary riders
//		};

class PSO : public RSAlgorithm {
	public:
		PSO(const std::string &);

		/* My overrides */
		virtual void match();
		virtual void handle_vehicle(const Vehicle &);
		virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

	private:
		Grid grid_;

		//PSO variables
		dict<CustId, vec_t<MutableVehicleSptr>> candidates_list;
		dict<VehlId, MutableVehicleSptr> vehicle_lookup;

		//dict<VehlId,vec_t<Driver>> drivers;

		DistInt get_cost(){

		  //return sop_insert(*cand, cust, sch, rte, Cargo::gtree()) - cand->route().cost();
		}
		bool pso_init(vec_t<int> & drivers);

		bool pso_body(vec_t<int> & drivers){

			return true;
		}

		/* Workspace variables */
		DistInt best_cost;
		vec_t<Stop> sch, best_sch;
		vec_t<Wayp> rte, best_rte;
		MutableVehicleSptr best_vehl;
		vec_t<MutableVehicleSptr> candidates;
		bool matched;
		tick_t timeout_0;

		void reset_workspace();
};

