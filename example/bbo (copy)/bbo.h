#include "libcargo.h"
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <random>
#include <numeric>
#include <algorithm>

using namespace cargo;

struct Driver{
			int capacity;		// capacity = real capacity - already fixed rides
			vec_t<int> rider;  //temporary riders
		};

class BBO : public RSAlgorithm {
	public:
		BBO(const std::string &);

		/* My overrides */
		virtual void match();
		virtual void handle_vehicle(const Vehicle &);
		virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

	private:
		std::random_device rd;
		Grid grid_;
		std::mt19937 gen;
		//BBO variables
		int NumberOfElites;
		float MutationProbability;
		int GenerationLimit;
		int PopulationSize;
		int problemDimension;
		vec_t <float> mu;
		vec_t <float> lambda;

		vec_t<dict<Customer, MutableVehicleSptr>> assignedRider;
		vec_t<Customer> unassignedRider;
		vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> solutions;
		vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> EliteSolutions;


		vec_t<long> solutionsCosts;
		vec_t<long> EliteCosts;

		vec_t<int> sortIndx;
		vec_t<long> minimuxCostPerGeneration;
		dict<Customer, vec_t<MutableVehicleSptr>> CandidateList;
		//dict<VehlId, MutableVehicleSptr> vehicle_lookup;
		//dict<CustId, Customer> cust_lookup;



		DistInt get_cost();
		void copyRider(int popIndxSrc, int popIndxDest, MutableVehicleSptr);
		void bbo_init();

		void bbo_body();
		void bbo_mutation();

		void costUpdate(int);
		void solution_show();

		/* Workspace variables */
		DistInt best_cost;
		vec_t<Stop> sch, best_sch;
		vec_t<Wayp> rte, best_rte;
		MutableVehicleSptr best_vehl;

		bool matched;
		tick_t timeout_0;

		void reset_workspace();
		float rand(){
			return rd()/float(rd.max());
		}
};

