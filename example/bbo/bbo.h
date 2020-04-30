#include "libcargo.h"
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <random>
#include <numeric>
#include <algorithm>
#include <dlib/clustering.h>

using namespace cargo;
using namespace dlib;

namespace MatchHistoryMap {

	typedef std::tuple<vec_t<TripId>, bool, vec_t<Stop>, vec_t<Wayp>, DistInt> value_capMatched;
	typedef std::tuple<Customer, MutableVehicleSptr> key_t;
	struct key_hash : public std::unary_function<key_t, std::size_t>
	{
		std::size_t operator()(const key_t& k) const
		{
			return std::get<0>(k).id() ^ std::get<1>(k)->id();
		}
	};

	struct key_equal : public std::binary_function<key_t, key_t, bool>
	{
		bool operator()(const key_t& v0, const key_t& v1) const
		{
			return (
			std::get<0>(v0).id() == std::get<0>(v1).id() &&
			std::get<1>(v0)->id() == std::get<1>(v1)->id()
			);
		}
	};
	typedef std::unordered_map<const key_t,value_capMatched,key_hash,key_equal> MatchHistory;
}
namespace rollBackHistoryMap {

	typedef std::tuple<vec_t<Stop>, vec_t<Wayp>> value;
	typedef std::tuple<MutableVehicleSptr> key_t;
	struct key_hash : public std::unary_function<key_t, std::size_t>
	{
		std::size_t operator()(const key_t& k) const
		{
			return std::get<0>(k)->id();
		}
	};

	struct key_equal : public std::binary_function<key_t, key_t, bool>
	{
		bool operator()(const key_t& v0, const key_t& v1) const
		{
			return (std::get<0>(v0) == std::get<0>(v1));
		}
	};
	typedef std::unordered_map<const key_t,value,key_hash,key_equal> rollBackHistory;;
}
using namespace MatchHistoryMap;
using namespace rollBackHistoryMap;

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
		virtual void end();

	private:

		const bool 		debugMode=0,
						rollBack=1,
						hybridInit=1,
						clustering_mode=1;


		const uint8_t 	hybridInitPercent=15,	//max is 100
						GenerationLimit=10,		//max is 255
						maxNumberOfElites=2,	//max is maxPopulationSize
						maxPopulationSize=20;	//max is 255

		const float 	MutationProbability= 0.04;

		MatchHistory matchHist;
		vec_t<int> sortedIdx;
		std::random_device rd;
		Grid grid_;
		vec_t<Grid> local_grid;
//		vec_t<Grid> elites_grid;
		std::mt19937 gen;
		//BBO variables
		int NumberOfElites;
		uint8_t PopulationSize; //maxpop=255

		int problemDimension;
		vec_t <float> mu;
		vec_t <float> lambda;

		vec_t<dict<Customer, MutableVehicleSptr>> assignedRider ;
		vec_t<dict<Customer, MutableVehicleSptr>> elitesAssignedRider;

		vec_t<dict<Customer, vec_t<MutableVehicleSptr>>> CandidateList;
		vec_t<dict<Customer, vec_t<MutableVehicleSptr>>> elitesCandidateList;
		vec_t<vec_t<Customer>> unassignedRider;
		vec_t<vec_t<Customer>> elitesUnassignedRider;
		vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> solutions;
		vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> elitesSolutions;
		vec_t<dict<VehlId, MutableVehicleSptr>> lookupVehicle;
		vec_t<dict<VehlId, MutableVehicleSptr>> elitesLookupVehicle;
		vec_t<long> solutionsCosts;
		vec_t<long> elitesCosts;
		vec_t<long> minimumCostPerGeneration;


		//dict<CustId, Customer> cust_lookup;



		DistInt get_cost();
		bool migrate_vehicle_based(int popIndxSrc, int popIndxDest, const MutableVehicleSptr , vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> &, vec_t<dict<Customer, MutableVehicleSptr>> & tempAssignedRider, vec_t<vec_t<Customer>> &tempUnassignedRider);
		void bbo_init();
		void insertElits();
		void bbo_body();
		void bbo_mutation();
		bool checkSCH(int solutionIdx, MutableVehicleSptr const & r);
		bool checkSCHTemp(int solutionIdx, MutableVehicleSptr const & r, vec_t<dict<Customer, MutableVehicleSptr>> const &tempAssignedRider, vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> const  &tempSolutions);
		void solutionsCostUpdate();
		void solution_show();
		bool Greedy_Assignment(int,const Customer &,
				vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & tempSolutions,
				vec_t<dict<Customer, MutableVehicleSptr>> &tempAssignedRider,
				vec_t<vec_t<Customer>> &tempUnassignedRider,
				rollBackHistory &);
		void ridersRandomlySelector(int indx, MutableVehicleSptr vehl, vec_t<Customer> & riders);
		void randXofY(int ridersCount, vec_t<Customer> & riders);
		void doSort();
		void commit();
		bool checkVehlStopsWithRiders(const vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & sol);
		bool checkVehlStopsDuplication(const MutableVehicleSptr &vehl, const Customer & cust);
		void selectElites();
		bool checkVehileConsistencyInAllStructures(	const dict<MutableVehicleSptr, vec_t<Customer>> & solutions,
															const dict<Customer, MutableVehicleSptr> & assignedRider,
															const dict<VehlId,MutableVehicleSptr> & lookupVehl,
															const dict<Customer, vec_t<MutableVehicleSptr>> & CandidateList	);
		MutableVehicleSptr GetMatchedHistory(const MutableVehicleSptr cand, const Customer & cust, vec_t<Stop> & sch, vec_t<Wayp> & rte, DistInt & cost, bool & is_matched);
		bool PutMatchedHistory(const MutableVehicleSptr cand, const Customer & cust,vec_t<Stop> & sch, vec_t<Wayp> & rte, DistInt & cost, const bool is_matched);
		bool UpdateMatchedStructures(const uint8_t indx, const MutableVehicleSptr cand, const Customer & cust);
		bool UpdateUnmatchedStructures(const uint8_t indx, const Customer & cust);
		//vec_t<Customer>::iterator searchRider(Customer const &a, vec_t<Customer> const &b);
		/* Workspace variables */
		DistInt best_cost;
		vec_t<Stop> sch, best_sch;
		vec_t<Wayp> rte, best_rte;
		MutableVehicleSptr best_vehl;
		MutableVehicleSptr initUsingGreedy(const Customer cust, const vec_t<MutableVehicleSptr> &candidates);
		MutableVehicleSptr initRandomely(const Customer cust, vec_t<MutableVehicleSptr> &candidates);
		bool matched;
		tick_t timeout_0;

		long init_cost;

		void reset_workspace();
		float rand(){
			return rd()/float(rd.max());
		}

		//####################Clustering Parts##############################

		typedef matrix<double,2,1> sample_type;

		typedef radial_basis_kernel<sample_type> kernel_type;
		void DriverClustering(vec_t<Point> data, int k);
		void AssignRidersToClusters(vec_t<Point> data, kcentroid<kernel_type> centroids);
		int ELBO_analysis(vec_t<Point> data);
		vec_t<Point> driver_points;
		};


