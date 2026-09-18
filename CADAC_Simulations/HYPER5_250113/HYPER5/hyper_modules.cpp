///////////////////////////////////////////////////////////////////////////////
//FILE: 'hyper_modules.cpp'
//Contains all modules of class 'Hyper'
//							aerodynamics()	hyper[30-39]
//							propulsion()	hyper[6-29]
//							forces()		round3[10]
//							targeting()		hyper[130-139]
//							seeker()		hyper[100-119]
//							guidance()		hyper[80-99]
//							control()		hyper[40-79]
//							intercept()		hyper[120-129]
//
//001122 Created by Peter H Zipfel
//001211 Introduced 'Variable' class to manage module-variables, PZi
//060512 Updated variable initialization, PZi
//060424 Included 'targeting' module, PZi
//231215 Building HYPER5 PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of aerodynamic module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[30-39]
// 
//Defining and initializing module-variables
//
//001226 Created by Peter H Zipfel
//231114 Building FALCON5 PZi
//231215 Building HYPER5 PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::def_aerodynamics()
{
	//Definition of module-variables
	hyper[30].init("cl",0,"Lift coefficient - ND","aerodynamics","out","scrn,plot");
	hyper[31].init("cd",0,"Drag coefficient - ND","aerodynamics","out","scrn,plot");
	hyper[32].init("cl_ov_cd",0,"Lift-over-drag ratio - ND","aerodynamics","diag","scrn,plot");
	hyper[33].init("area",0,"Aerodynamic reference area - m^2","aerodynamics","data","");
	hyper[34].init("cla",0,"Lift slope derivative - 1/deg","aerodynamics","out","scrn,plot");
	hyper[35].init("cn",0,"Normal force coefficient - ND","aerodynamics","dia","scrn,plot");
	hyper[36].init("ca",0,"Axial force coefficient - ND","aerodynamics","dia","scrn,plot");
}	
//$$$//////////////////////////////////////////////////////////////////////////
//Aerodynamic module
//Member function of class 'Hyper'
// 
// Ref: "Generic Hypersonic Vehicles for Conceptual Design Analyses"
//       Brent Ruttle, Jacob Stork, Glenn Liston, AFRL, Sept 2012
//		  Configuration 'Roadrunner 3X'
// 
// Reference area = 11.6986 m^2
// Length = 7.74 m
// Wingspan = 2.58 m 
//
//001023 Created by Peter Zipfel
//001227 Upgraded to module-variable arrays, PZi
//231122 Building FALCON5 PZi
//231215 Building HYPER5 PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::aerodynamics()
{
	//local variables
	double cn(0);
	double ca(0);
	double cl(0);
	double cd(0);
	double cla(0);
	double cl_ov_cd(0);
	
	//localizing module-variables
	//input from other modules
	double time=round3[0].real();
	double mach=round3[14].real();
	double alphax=hyper[51].real();
	//-------------------------------------------------------------------------
	//aeroballistic conventions	
	cn=aerotable.look_up("cn_rr3x_vs_alphax_mach",alphax,mach);
	ca=aerotable.look_up("ca_rr3x_vs_alphax_mach",alphax,mach);
	//aircraft conventions
	double alpha=alphax*RAD;
	cd=cn*sin(alpha)+ca*cos(alpha);
	cl=cn*cos(alpha)-ca*sin(alpha);
	cl_ov_cd=cl/cd;

	//computing lift-alpha derivative (1/deg) for controllers
	double cnp=aerotable.look_up("cn_rr3x_vs_alphax_mach",alphax+2,mach);
	double cnn=aerotable.look_up("cn_rr3x_vs_alphax_mach",alphax-2,mach);
	double cap=aerotable.look_up("ca_rr3x_vs_alphax_mach",alphax+2,mach);
	double can=aerotable.look_up("ca_rr3x_vs_alphax_mach",alphax-2,mach);
	double cna=(cnp-cnn)/4;
	double caa=(cap-can)/4;
	cla=cna*cos(alpha)-caa*sin(alpha);
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	hyper[30].gets(cl);
	hyper[31].gets(cd);
	hyper[34].gets(cla);
	//diagnostics
	hyper[32].gets(cl_ov_cd);
	hyper[35].gets(cn);
	hyper[36].gets(ca);
}	
///////////////////////////////////////////////////////////////////////////////
//Definition of 'propulsion' module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[10-29]
// 
// Hypersonic propulsion
// Ref: "Generic Hypersonic Vehicles for Conceptual Design Analyses"
//       Brent Ruttle, Jacob Stork, Glenn Liston, AFRL Sept 2012
// 
// Mass properties: gross mass  1976 kg
//                  usable fuel mass 624 kg
//
// mrpop = 0 No thrusting
//       = 1 Hypersonic propulsion phi (throttle) command
//       = 2 Hypersonic propulsion autothrottle
//		 = 3 Maintaining constant phi (throttle)
//
// This module performs the following functions:
// (1) Provides propulsion input capture area and specific impulse
// (2) Initializes vehicle mass properties
// (3) Sets up fuel mass integration variable
//
//100505 Created by Peter H Zipfel
//231215 Building HYPER5 PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::def_propulsion()
{
	//Definition and initialization of module-variables
    hyper[6].init("phi_const",0,"Constant phi input (throttle) - ND","propulsion","data","");
    hyper[7].init("tlag",0,"Time constant for throttle lag (phi) - sec","propulsion","data","");
	hyper[8].init("phis",0,"Lagged throttle phi - ND","propulsion","state","plot");
    hyper[9].init("phisd",0,"Lagged throttle phi derivative - ND/s","propulsion","state","plot");
	hyper[10].init("mprop","int",0,"=0:none;=1:fixed-phi;=2:auto-phi;=3:keep-phi","propulsion","data","plot");
    hyper[11].init("aintake",0,"Propulsion intake area - m^2","propulsion","data","");
    hyper[12].init("phi",0,"Equivalence ratio (throttle) - ND","propulsion","data/diag","scrn,plot");
    hyper[13].init("phi_max",0,"Max phi - ND","propulsion","data","");
    hyper[14].init("qhold",0,"Dynamic pressure hold command - Pa","propulsion","data","");
    hyper[15].init("mass",0,"Vehicle mass - kg","propulsion","out","scrn,plot");
    hyper[16].init("mass0",0,"Initial gross mass - kg","propulsion","data","");
    hyper[17].init("cin",0,"Effective inlet area ratio - ND","propulsion","dia","scrn,plot");
    hyper[18].init("tq",0,"Autothrottle time constant - sec","propulsion","data","");
    hyper[19].init("phi_min",0,"Idle phi - ND","propulsion","data","");
	hyper[21].init("fmass0",0,"Initial fuel mass in stage - kg","propulsion","data","");
    hyper[22].init("fmasse",0,"Fuel mass expended (zero initialization required) - kg","propulsion","state","");
    hyper[23].init("fmassd",0,"Fuel mass expended derivative - kg/s","propulsion","state","");
    hyper[24].init("thrst_stoch",0,"Stochiometric thrust (phi=1) - N","propulsion","dia","plot");
    hyper[25].init("spi",0,"Specific impulse - sec","propulsion","diag","scrn,plot");
    hyper[26].init("thrust",0,"Thrust - N","propulsion","out","scrn,plot");
    hyper[27].init("mass_flow",0,"Mass flow through hypersonic engine - kg/s","propulsion","diag","");
    hyper[28].init("fmassr",0,"Remaining fuel mass - kg","propulsion","diag","scrn,plot");
    hyper[29].init("thrst_req",0,"thrust required - N","propulsion","diag","scrn,plot");
}	
///////////////////////////////////////////////////////////////////////////////
//Propulsion initialization module
//Member function of class 'Hyper'
// Initializes mass properties
//
//100505 Created by Peter H Zipfel
//231215 Building HYPER5 PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::init_propulsion()
{
	//local module-variables
	double mass(0);

	//localizing module-variables
	double mass0=hyper[16].real();
	//-------------------------------------------------------------------------
	mass=mass0;
	//-------------------------------------------------------------------------
	//loading module-variables
	//initialization
	hyper[15].gets(mass);
}
//$$$/////////////////////////////////////////////////////////////////////////////
//Propulsion module
//Member function of class 'Hyper'
// Calculates engine thrust
// Provides dynamic pressure controller 
//  using the equivalent ratio phi=(fuel/air demanded)/(fuel/air stochiometric) as throttle 
//
// mrpop = 0 No thrusting
//       = 1 Hypersonic propulsion phi (throttle) command
//       = 2 Hypersonic propulsion autothrottle
//		 = 3 From now on maintaining constant phi (throttle)
//
//100505 Created by Peter H Zipfel
//231215 Building HYPER5 PZi
//240415 Introduced throttle lag PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::propulsion(double int_step)
{
	//local module-variables
	double spi(0);
	double thrust(0);
	double mass_flow(0);
	double cin(0);
	double phi(0);
	double thrst_req(0);
	double thrst_stoch(0);

	//localizing module-variables
	//input data
	int mprop=hyper[10].integer();
	double tlag=hyper[7].real(); 
	double aintake=hyper[11].real(); 
	double phi_const=hyper[6].real(); 
	double phi_max=hyper[13].real();
	double qhold=hyper[14].real();
	double mass0=hyper[16].real();
	double fmass0=hyper[21].real();
	double tq=hyper[18].real();
	double phi_min=hyper[19].real();
	//getting saved variable
	double mass=hyper[15].real();
	double fmassr=hyper[28].real();
	//state variable
	double phis=hyper[8].real();
	double phisd=hyper[9].real();
	double fmasse=hyper[22].real();
	double fmassd=hyper[23].real();
	//input from other modules
	double time=round3[0].real();
	double rho=round3[12].real();
	double pdynmc=round3[13].real();
	double mach=round3[14].real();
	double dvbe=round3[25].real();
	double ca=hyper[36].real();
	double area=hyper[33].real();
	double alphax=hyper[51].real();
	//-------------------------------------------------------------------------
	//making thrust calculations only if engine is on
	if(mprop>0){

		//hypersonic propulsion table look-up
		if(mprop){
			//intake-capture-area-factor table look-up
			cin=proptable.look_up("cin_vs_alphax_mach",alphax,mach);
			//specific impulse table look-up

			spi=proptable.look_up("spi_vs_mach_phi_alphax",mach,phi,alphax);
		}
		//hypersonic propulsion with fixed phi
		if(mprop==1){
			thrust=0.0676*phi_const*spi*AGRAV*rho*dvbe*cin*aintake;
		}
		//hypersonic propulsion controlling dynamic pressure by setting phi (throttle)
		if(mprop==2){
			thrst_stoch=0.0676*spi*AGRAV*rho*dvbe*cin*aintake;
			thrst_req=area*ca*qhold;
			double phi_req=thrst_req/thrst_stoch;
			double gainq=2*mass/(rho*dvbe*thrst_stoch*tq);
			double ephi=gainq*(qhold-pdynmc);
			phi=phi_req+ephi; 

			//introducing lag with time constant tlag and gain 1
			double phisd_new=(phi-phis)/tlag;
			phis=integrate(phisd_new,phisd,phis,int_step);
			phisd=phisd_new;
			phi=phis;

			//phi limiters
			if (phi<phi_min)phi=phi_min;
			if (phi>phi_max) phi=phi_max;

			//calculating thrust
			spi=proptable.look_up("spi_vs_mach_phi_alphax",mach,phi/0.0676,alphax);
			thrust=0.0676*phi*spi*AGRAV*rho*dvbe*cin*aintake;
		}
		//maintaining contstant equivalence ratio phi (throttle)
		if(mprop==3){
			phi; //maintaining previous phi value 
			spi=proptable.look_up("spi_vs_mach_phi_alphax",mach,phi/0.0676,alphax);
			thrust=0.0676*phi*spi*AGRAV*rho*dvbe*cin*aintake;
		}
		//calculating fuel consumption
		double fmassd_next=thrust/(spi*AGRAV);
		fmasse=integrate(fmassd_next,fmassd,fmasse,int_step);
		fmassd=fmassd_next;

		//calculating vehicle mass, mass flow, and fuel mass remaining
		mass=mass0-fmasse;
		fmassr=fmass0-fmasse;

		//diagnostics
		mass_flow=thrust/(AGRAV*spi);

		//shutting down engine when all fuel is expended
		if(fmassr<=0)
			mprop=0;
	}
	//no thrusting
	if(mprop==0){
		fmassd=0;
		thrust=0;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	hyper[8].gets(phis);
	hyper[9].gets(phisd);
	hyper[22].gets(fmasse);
	hyper[23].gets(fmassd);
	//saving variable
	hyper[12].gets(phi);
	//output to other modules
	hyper[15].gets(mass);
	hyper[26].gets(thrust);
	//diagnostics
	hyper[10].gets(mprop);
	hyper[17].gets(cin);
	hyper[24].gets(thrst_stoch);
	hyper[25].gets(spi);
	hyper[27].gets(mass_flow);
	hyper[28].gets(fmassr);
	hyper[29].gets(thrst_req);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of force module-variables
// Member function of class 'Hyper'
//
//Note that FSPV is entered into the round3[10] array because it is needed
// for the newton module, which is a member of the 'Round3' class
//		
//001129 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
void Hyper::def_forces()
{
	//Definition of module-variables
	round3[10].init("FSPV",0,0,0,"Specific force in V-coord - m/s^2","forces","out","plot");
}

//$$$//////////////////////////////////////////////////////////////////////////
//Force Module 
//Member function of class 'Hyper' 
//Calulates the total force acting on the vehicle
//
//000623 Created by Michael Chiaramonte
//000724 Function calls have been removed, Michael Horvath
//001227 Upgraded to module-variable arrays, PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::forces()
{
	//local module-variable
	Matrix FSPV(3,1);

	//localizing module-variables	
	//input from other modules
	double pdynmc=round3[13].real();
	double cl=hyper[30].real();
	double cd=hyper[31].real();
	double area=hyper[33].real();
	double thrust=hyper[26].real();
	double mass=hyper[15].real();
	double alphax=hyper[51].real();
	double phimvx=hyper[52].real();
	//-------------------------------------------------------------------------
	double phimv=phimvx*RAD;
	double alpha=alphax*RAD;

	double fspv1=(-pdynmc*area*cd+thrust*cos(alpha))/mass; 
	double fspv2=sin(phimv)*(pdynmc*area*cl+thrust*sin(alpha))/mass;
	double fspv3=-cos(phimv)*(pdynmc*area*cl+thrust*sin(alpha))/mass;
	
	FSPV.assign_loc(0,0,fspv1);
	FSPV.assign_loc(1,0,fspv2);
	FSPV.assign_loc(2,0,fspv3);
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	round3[10].gets_vec(FSPV);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'targeting' module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[130-139]
//		
//010813 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Hyper::def_targeting()
{
	//definition of module-variables
	hyper[130].init("mtargeting","int",0,"Satellite targeting mode switch - ND","targeting","data","scrn,plot");
	hyper[131].init("del_radius",0,"Increase in Earth's radius for 'visibility' - m","targeting","data","");
	hyper[132].init("clost_tgt_slot","int",0,"Closest of satellite tracked targets ' - ND","targeting","out","");
	hyper[133].init("tgtng_sat_slot","int",0,"Satellite providing targeting ' - ND","targeting","out","");
}
//$$$//////////////////////////////////////////////////////////////////////////  
//Targeting module
//Member function of class 'Hyper'
//
//mtargeting:
//			= 0 no satellite targeting taking place
//			= 1 targeting enabled
//NOTE:
//The 'Target'-object variables 'lonx','latx','alt','dvbe','psivgx','thtvgx', SBII
// must be located in 'combus' at 'Packet data[i]', i=2,3,4,5,6,7,10 respectively.
//These locations are determined by the sequence of the "com"-key entry
// in the module-variable array 'round3[]'
//		
//010813 Created by Peter H Zipfel
//070313 Added output to console, PZi
///////////////////////////////////////////////////////////////////////////////	
void Hyper::targeting(Packet *combus,int vehicle_slot,int num_vehicles,int num_target
					   ,int num_satellite)
{
	//local variables
	double range(0);
	bool satellite_found=false;

	Variable *data_t;
	static int out_count(0);

	//local module-variables
	int clost_tgt_slot(0);
	int tgtng_sat_slot(0);
	double wp_lonx(0);
	double wp_latx(0);
	double wp_alt(0);

	//localizing module-variables
	//input data
	int mtargeting=hyper[130].integer();
	//-------------------------------------------------------------------------
	//returning if no targeting is taking place
	if(mtargeting==0) return;

	//determining satellites that can provide targeting data and identify them in 'visibility[]'
	targeting_satellite(combus,num_vehicles);

	//is there a satellite that can provide targeting info? 
	//if there is more than one, the first one in the vehicle slot# sequence is taken
	//j target counter; i vehicle counter, k satellite counter	for(int k=0;k<num_satellite;k++)
	for(int k=0;k<num_satellite;k++)
	{
		if(!satellite_found)
		{
			int active=visibility[k].tracking;
			if(active)
			{
				satellite_found=true;
				tgtng_sat_slot=visibility[k].vehicle_slot;
				//building 'grnd_range[]' (ranges to all targets)
				targeting_grnd_ranges(combus,num_vehicles);

				//determining closest target
				range=BIG;

				int j=0;

				for(int i=0;i<num_vehicles;i++)
				{
					string id=combus[i].get_id();
					int loc=(int)id.find("t");
					if(!loc)
					{
						//getting ground ranges to targets
						double new_range=grnd_range[j];
						j++;
						if(new_range<range)
						{
							range=new_range;
							clost_tgt_slot=i;
						}
					}
				}//closest target determined
			}//first satellite that is able to provide targeting info
		}//satellite found
	}//all satellites were interrogated
	if(satellite_found)
	{
		//loading the closest target position for the 'guidance' module
		data_t=combus[clost_tgt_slot].get_data();
		wp_lonx=data_t[2].real();
		wp_latx=data_t[3].real();
		wp_alt=data_t[4].real();
		string target_id=combus[clost_tgt_slot].get_id();
		string satellite_id=combus[tgtng_sat_slot].get_id();
		out_count++;
		if(out_count==1) cout<<" *** Satellite_"<<satellite_id<<" tracks Target_"<<target_id<<" *** \n";
	}
	else
	{
		out_count++;
		if(out_count==1) cout<<" *** No satellite can track targets *** \n";
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	hyper[132].gets(clost_tgt_slot);
	hyper[133].gets(tgtng_sat_slot);
	hyper[85].gets(wp_lonx);
	hyper[86].gets(wp_latx);
	hyper[87].gets(wp_alt);
}
///////////////////////////////////////////////////////////////////////////////  
//Determining whether satellites are visible from 'this' hyper missile and can track targets
// Checking for:	(1) are 'this' hyper missile and satellites in line-of-sight
//					(2) are first target of 'vehicle_list' and any satellite in line-of-sight 
//The status of all satellites (targeting or not-targeting) is stored in 'Targeting visibility[]'
//
//Assumption:
//  If the first target can be seen, so all targets can be seen by the satellite;
//  therefore target visibility is soley based on first target 
//Requirement:
// The 'Target'-object variables SBII(3x1) must be located in 'combus' at
//  'Packet data[i]', i=10;
//  this location is determined by the sequence of the "com"-key entry
//   in the module-variable array 'round3[]' 
//
//Output: 'Targeting Hyper::visibility[]'; entry: not visible = '0', visible = '1'
//		
//010813 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Hyper::targeting_satellite(Packet *combus,int num_vehicles)
{
	//local variables
	int k(0);
	int i(0);
	string id;
	Variable *data_first_target; //module-variable data of first target 
	Variable *data_sat;
	Matrix STII(3,1);
	Matrix SSII(3,1);
	double grazing_angle(0);
	double satellite_missile_angle(0);
	double satellite_target_angle(0);
	double alt_crit(0);
	bool first_target=false;

	//localizing module-variables
	//input data
	double del_radius=hyper[131].real();
	//input from other modules
	Matrix SBII=round3[35].vec();
	//-------------------------------------------------------------------------
	double radius=REARTH+del_radius;
	//locating first target in 'combus' and its inertial vector
	for(i=0;i<num_vehicles;i++)
	{
		if(!first_target)
		{
			id=combus[i].get_id();
			int loc=(int)id.find("t"); //Schildt p.687, returns index (0, first letter) of match
			if(!loc)
			{
				first_target=true;
				data_first_target=combus[i].get_data();
				STII=data_first_target[10].vec();
			}
		}
	}
	//cycling through the vehicles, locating  satellites
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		int loc=(int)id.find("s"); //Schildt p.687, returns index (0, first letter) of match

		if(!loc)
		//satellite is found
		{
			data_sat=combus[i].get_data();
			SSII=data_sat[10].vec();
		//
			//Determining clear line-of-sight between satellite and hyper
			double dsi=SSII.absolute();
			grazing_angle=acos(radius/dsi);
			satellite_missile_angle=angle(SBII,SSII);

			if(satellite_missile_angle<grazing_angle)
			//always visible
			{
				visibility[k].tracking=1;
				visibility[k].vehicle_slot=i;
				k++;
			}
			else
			//visible if altitude of hyper is sufficient
			{
				//critical altitude
				double radius_crit=0;
				double dum=cos(satellite_missile_angle-grazing_angle);
				if(fabs(dum)>EPS) radius_crit=radius/dum;				
				alt_crit=radius_crit-REARTH;
				double dbi=SBII.absolute();
				if(dbi>radius_crit)
				{
					visibility[k].tracking=1;
					visibility[k].vehicle_slot=i;
					k++;
				}
				else
				{
					visibility[k].tracking=0;
					visibility[k].vehicle_slot=i;
					k++;
				}
			}
			//determining if there is NO clear line-of-site to first target
			// then reset 'tracking=0'
			satellite_target_angle=angle(SBII,STII);
			if(satellite_target_angle>grazing_angle)
			{
				visibility[k-1].tracking=0;
			}
		}
	}//all satellites evaluated and stored in 'visibility[]'
}
///////////////////////////////////////////////////////////////////////////////  
//Calculating ground distances of hyper missile to all targets
// same function as 'seeker_grnd_ranges()'
//
//Assumption:
//	The 'Target'-object variables 'lonx' and 'latx' must be located in 'combus' at
// 'Packet data[i]', i=1 and i=2, respectively
//These locations are determined by the sequence of the "com"-key entry
// in the module-variable array 'round3[]' 
//
//Output: 'Hyper::grnd_range[]', ground range from current 'Hyper' object to all 'Target' objects
//		
//010813 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Hyper::targeting_grnd_ranges(Packet *combus,int num_vehicles)
{
	//local variables
	int k(0);
	string id;

	//localizing module-variables
	//input from other modules
	double lonx=round3[19].real();
	double latx=round3[20].real();
	//-------------------------------------------------------------------------
	double lon_c=lonx*RAD;
	double lat_c=latx*RAD;

	for(int i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		int loc=(int)id.find("t");
		if(!loc)
		{
			Variable *data_c2=combus[i].get_data();
			double lonx_t=data_c2[2].real();
			double latx_t=data_c2[3].real();

			double lon_t=lonx_t*RAD;
			double lat_t=latx_t*RAD;

			//calculating separation distance over round earth
			double dum=sin(lat_t)*sin(lat_c)+cos(lat_t)*cos(lat_c)*cos(lon_t-lon_c);

			//load into 'grnd_range' array 
			grnd_range[k]=REARTH*acos(dum);
 			k++;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//Definition of seeker module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[100-119]
//		
//010221 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Hyper::def_seeker()
{
	//definition of module-variables
	hyper[100].init("mseeker","int",0,"Mode switch - ND","seeker","data/save","scrn");
	hyper[101].init("acq_range",0,"Seeker acquisition range - m","seeker","data","");
	hyper[105].init("range_go",0,"LOS range-to-go - m","seeker","out","plot,scrn");
	hyper[106].init("STBG",0,0,0,"Displacement of target wrt vehicle - m","seeker","out","plot");
	hyper[107].init("WOEB",0,0,0,"LOS rate wrt earth in body coord - rad/s","seeker","out","");
	hyper[108].init("closing_speed",0,"closing velocity - m/s^2","seeker","out","");
	hyper[109].init("time_go",0,"Time-to-go - s","seeker","out","plot,scrn");
	hyper[110].init("psisbx",0,"Seeker azimuth angle - deg","seeker","out","plot,scrn");
	hyper[111].init("thtsbx",0,"Seeker elevation angle - deg","seeker","out","plot,scrn");
	hyper[112].init("targ_com_slot","int",0,"Slot # of target,attacked by hyper missile, in 'combus' - ND","seeker","save","");
	hyper[113].init("UTBB",0,0,0,"Unit LOS vector - ND","seeker","out","");
	hyper[114].init("acquisition","int",0,"Acquisition flag (initially false) - ND","seeker","init/save","scrn");
}
//$$$/////////////////////////////////////////////////////////////////////////////  
//Seeker module
//Member function of class 'Hyper'
//
//mseeker:
//			= 0 no seeker 
//			= 1 seeker enable
//			= 2 seeker acquisition
//			= 3 seeker tracking
//
//NOTE:
//The 'Target'-object variables 'lonx','latx','alt','psivgx','thtvgx','VTEG','STII'
// must be located in 'combus' at 'Packet data[i]', i=2,3,4,6,7,9,10 respectively.
//These locations are determined by the sequence of the "com"-key entry
// in the module-variable array 'round3[]'
//		
//010221 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////	
void Hyper::seeker(Packet *combus,int vehicle_slot,int num_vehicles,int num_target)
{
	//local variables
	double range(0);
	string target_id;
	Variable *data_t;
	double lonx_t(0);  //target longitude
	double latx_t(0);  //target latitude
	double alt_t(0);   //target altitude
	double psivgx_t(0);//target heading angle
	double thtvgx_t(0);//target incline angle
	Matrix STBI(3,1);  //displacement of target wrt hyper missile in inertial coordinates
	double inv_dtb(0); //inverse of range-to-go
	Matrix VTBG(3,1);  //velocity of target wrt missle
	Matrix VBTG(3,1);  //velocity of hyper missile wrt target
	Matrix DUM33(3,3);
	Matrix DUM3(3,1);
	Matrix UTBG(3,1);  //unit LOS vector
	Matrix POLAR(3,1);
	Matrix TGI(3,3);   //T.M. of geographic, wrt inertial coord
	Matrix STII(3,1);  //Inertial target position
	Matrix VTEG(3,1);  //Target geographic velocity

	//local module-variables
	double range_go(0);
	Matrix STBG(3,1);
	Matrix WOEB(3,1);
	double closing_speed(0);
	double time_go(0);
	double psisbx(0);
	double thtsbx(0);
	Matrix UTBB(3,1);

	//localizing module-variables
	//input data
	int mseeker=hyper[100].integer();
	double acq_range=hyper[101].real();
	int acquisition=hyper[114].integer();
	//input from other modules
	double time=round3[0].real();
	Matrix TIG=round3[23].mat();
	Matrix VBEG=round3[32].vec();
	Matrix SBII=round3[35].vec();
	Matrix TBG=hyper[46].mat();
	//restore saved values
	int mcontrol=hyper[40].integer();
	int mguidance=hyper[80].integer();
	int targ_com_slot=hyper[112].integer();
	//-------------------------------------------------------------------------
	//returning if no seeker
	if(mseeker==0) return;

	//building Hyper::grnd_range[] member array (ranges to all targets)
	seeker_grnd_ranges(combus,num_vehicles);

	//acquisition has not occured yet and seeker is enabled
	if(!acquisition&&(mseeker==1))
	{
		for(int j=0;j<num_target;j++)
		{
			//getting groundranges to targets
			range=grnd_range[j];

			//picking the target within acquisition range
			if(range<acq_range)
			{
				acquisition=1;
				//seeker starts tracking
				mseeker=3;

				//building target id = t(j+1)
				char number[4];	
				sprintf(number,"%i",j+1);
				target_id="t"+string(number);

				//finding slot 'i' of target in 'combus' (same as in vehicle_list)
				for(int i=0;i<num_vehicles;i++)
				{
					string id=combus[i].get_id();
					if (id==target_id)
					{						
						targ_com_slot=i;
					}
				}
				//getting hyper missile # (current vehicle = current'combus' slot) and target #
				string id_targ=combus[targ_com_slot].get_id();
				string id_missl=combus[vehicle_slot].get_id();

				//writing seeker acquisition message to console
				cout<<"\n"<<" *** Acquisition by Missile_"<<id_missl<<" of Target_"<<target_id
					<<" at time = "<<time<<" sec ***\n\n";
			}
		}
	}//acquisition has occurred and target-packet slot # is identified in 'combus'

	//seeker is tracking
	if(mseeker==3)
	{
		// getting data of target being tracked (tied to a special sequence in 'combus.data')
		data_t=combus[targ_com_slot].get_data();
		lonx_t=data_t[2].real();
		latx_t=data_t[3].real();
		alt_t=data_t[4].real();
		psivgx_t=data_t[6].real();
		thtvgx_t=data_t[7].real();
		VTEG=data_t[9].vec();
		STII=data_t[10].vec();

		//calculating relative hyper-target displacement in hyper geo. coord.
		STBI=STII-SBII;
		TGI=TIG.trans();
		STBG=TGI*STBI;

		//calculating range-to-go
		range_go=STBG.absolute();

		//calculating unit vector of LOS and converting to skew-symmetric form
		inv_dtb=1/range_go;
		UTBG=STBG*inv_dtb;

		//calculating LOS rate in body coordinates
		VTBG=VTEG-VBEG;
		WOEB=TBG*UTBG.skew_sym()*VTBG*inv_dtb;

		//calculating closing speed (pos if closing on target, otherwise negative)
		VBTG=VTBG*(-1);
		closing_speed=UTBG^VBTG;

		//calculating time-to-go
		time_go=range_go/closing_speed;

		//LOS angles wrt body frame
		UTBB=TBG*UTBG;
		POLAR=UTBB.pol_from_cart();
		psisbx=POLAR.get_loc(1,0)*DEG;
		thtsbx=POLAR.get_loc(2,0)*DEG;

	//-------------------------------------------------------------------------
		//loading module-variables
		//(done in this block so the initial zero values are maintained until overwritten here)
		//output to other modules
		hyper[105].gets(range_go);
		hyper[106].gets_vec(STBG);
		hyper[107].gets_vec(WOEB);
		hyper[108].gets(closing_speed);
		hyper[109].gets(time_go);
		hyper[110].gets(psisbx);
		hyper[111].gets(thtsbx);
		hyper[113].gets_vec(UTBB);
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving variables
	hyper[40].gets(mcontrol);
	hyper[80].gets(mguidance);
	hyper[112].gets(targ_com_slot);
	hyper[114].gets(acquisition);
	//output to other modules
	hyper[100].gets(mseeker);
}

///////////////////////////////////////////////////////////////////////////////  
//Calculating ground distances to all targets
//
//Assumptions:
//	The 'Target'-object variables 'lonx' and 'latx' must be located in 'combus' at
// 'Packet data[i]', i=1 and i=2, respectively
//These locations are determined by the sequence of the "com"-key entry
// in the module-variable array 'round3[]' 
//
//Output: Hyper::grnd_range[], ground range from current 'Hyper' object to all 'Target' objects
//		
//010215 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Hyper::seeker_grnd_ranges(Packet *combus,int num_vehicles)
{
	//local variables
	int k=0;
	string id;

	//localizing module-variables
	double lonx_c=round3[19].real();
	double latx_c=round3[20].real();

	double lon_c=lonx_c*RAD;
	double lat_c=latx_c*RAD;

	for(int i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		int loc=(int)id.find("t");
		if(!loc)
		{
			Variable *data_c2=combus[i].get_data();
			double lonx_t=data_c2[2].real();
			double latx_t=data_c2[3].real();

			double lon_t=lonx_t*RAD;
			double lat_t=latx_t*RAD;

			//calculating separation distance over round earth
			double dum=sin(lat_t)*sin(lat_c)+cos(lat_t)*cos(lat_c)*cos(lon_t-lon_c);

			//load into 'grnd_range' array 
			grnd_range[k]=REARTH*acos(dum);
 			k++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Definition of guidance module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[80-99]
//		
//010209 Created by Peter H Zipfel
//240407 Modified for HYPER5, PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::def_guidance()
{
	//definition of module-variables
	hyper[80].init("mguidance","int",0,"Switch for guidance options - ND","guidance","data","scrn");
	hyper[81].init("pronav_gain",0,"Proportional navigation gain - ND","guidance","data","");
	hyper[82].init("line_gain",0,"Line guidance gain - 1/s","guidance","data","");
	hyper[83].init("nl_gain_fact",1,"Nonlinear gain factor - ND","guidance","data","");
	hyper[84].init("decrement",0,"distance decrement - m","guidance","data","");
	hyper[85].init("wp_lonx",0,"Longitude of way point - deg","guidance","data","");
	hyper[86].init("wp_latx",0,"Latitude of way point - deg","guidance","data","");
	hyper[87].init("wp_alt",0,"Altitude of way point - m","guidance","data","");
	hyper[88].init("psifgx",0,"Heading line-of-attack angle - deg","guidance","data","");
	hyper[89].init("thtfgx",0,"Pitch line-of-attack angle - deg","guidance","data","");
	hyper[90].init("point_gain",0,"Point guidance gain - 1/s","guidance","data","");
	hyper[91].init("wp_sltrange",999999,"Range to waypoint - m","guidance","dia","scrn,plot");
	hyper[92].init("nl_gain",0,"Nonlinear gain - rad","guidance","dia","");
	hyper[93].init("VBEO",0,0,0,"Vehicle velocity in LOS coordinats - m/s","guidance","dia","");
	hyper[94].init("VBEF",0,0,0,"Vehicle velocity in LOA coordinats - m/s","guidance","dia","");
	hyper[95].init("wp_grdrange",999999,"Ground range to waypoint - m","guidance","dia","scrn,plot");
	hyper[96].init("SWBG",0,0,0,"Vehicle wrt waypoint/target in geo coor - m","guidance","out","");
	hyper[97].init("rad_min",0,"Minimum arc radius, calculated - m","guidance","dia","");
	hyper[98].init("bias",0,"Pronav accel bias, positive up - m/s^2","guidance","data","");
	hyper[99].init("wp_flag","int",0,"=1:closing on trgt; =-1:fleeting; =0:outside - ND","guidance","dia","");
}
//$$$//////////////////////////////////////////////////////////////////////////  
//Guidance module
//Member function of class 'Hyper'
//
//mguidance:
//			= 30 line-guidance lateral, with mcontrol 46 
//			= 03 line-guidance in pitch 
//			= 33 line-guidance lateral and in pitch, with mcontrol=44
//			= 40 point-guidance lateral, with mcontrol 46
//			= 43 point-guidance lateral, line-guidance in pitch, with mcontrol=44
//			= 44 point-guidance lateral and in pitch , with mcontrol=44 -> used in HYPER5
//			= 60 pro-nav lateral  
//			= 06 pro-nav in pitch 
//			= 66 pro-nav lateral and in pitch, with mcontrol=44 -> used in HYPER5
//			= 70 arc-guidance lateral, with mcontrol=36 -> used in HYPER5
// 
//010209 Created by Peter H Zipfel
//010820 Added point guidance capability, PZi
//010823 New waypoint arc guidance, PZi
//240106 Additional guidance option for HYPER5, PZi
///////////////////////////////////////////////////////////////////////////////	
void Hyper::guidance()
{
	//local variables
	Matrix APNB(3,1);
	Matrix ALGV(3,1);
	Matrix APGV(3,1);

	//local module-variables
	double ancomx(0);
	double alcomx(0);

	//localizing module-variables
	//input data
	int mguidance=hyper[80].integer();
	//input from other modules
	double grav=round3[11].real();
	double phicx=hyper[53].real();
	double anposlimx=hyper[58].real();
	double anneglimx=hyper[59].real();
	double allimx=hyper[72].real();
	double range_go=hyper[105].real();
	//-------------------------------------------------------------------------
	//returning if no guidance
	if(mguidance==0)
	{
		alcomx=0;
		ancomx=0;
		return;
	}
	//lateral line guidance
	if(mguidance==30)
	{
		ALGV=guidance_line();
		alcomx=ALGV.get_loc(1,0)/grav;
	}
	//pitch line guidance
	if(mguidance==3)
	{
		ALGV=guidance_line();
		alcomx=0;
		ancomx=-ALGV.get_loc(2,0)/grav;
	}
	//line guidance lateral and pitch
	if(mguidance==33)
	{
		ALGV=guidance_line();
		alcomx=ALGV.get_loc(1,0)/grav;
		ancomx=-ALGV.get_loc(2,0)/grav;
	}
	//lateral pro-nav
	if(mguidance==60)
	{
		APNB=guidance_pronav();
		alcomx=APNB.get_loc(1,0)/grav;
		ancomx=0;
	}
	//pitch pro-nav
	if(mguidance==6)
	{
		APNB=guidance_pronav();
		alcomx=0;
		ancomx=-APNB.get_loc(2,0)/grav;
	}
	//pro-nav lateral and pitch
	if(mguidance==66)
	{
		APNB=guidance_pronav();
		alcomx=APNB.get_loc(1,0)/grav;
		ancomx=-APNB.get_loc(2,0)/grav;
	}
	//point guidance lateral and line guidance pitch
	if(mguidance==43)
	{
		ALGV=guidance_line();
		APGV=guidance_point();
		alcomx=APGV.get_loc(1,0)/grav;
		ancomx=-ALGV.get_loc(2,0)/grav;
	}
	//point guidance lateral
	if(mguidance==40)
	{
		APGV=guidance_point();
		alcomx=APGV.get_loc(1,0)/grav;
	}
	//point guidance lateral and pitch
	if(mguidance==44)
	{
		APGV=guidance_point();
		alcomx=APGV.get_loc(1,0)/grav;
		ancomx=-APGV.get_loc(2,0)/grav;
	}
	//arc guidance lateral
	if(mguidance==70)
	{
		phicx=guidance_arc();
	}

	//limiting normal load factor command
	if(ancomx>anposlimx) ancomx=anposlimx;
	if(ancomx<anneglimx) ancomx=anneglimx;

	//limiting lateral load factor command
	if(alcomx>allimx) alcomx=allimx;
	if(alcomx<-allimx) alcomx=-allimx;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	hyper[53].gets(phicx);
	hyper[70].gets(ancomx);
	hyper[71].gets(alcomx);
}

//%%%///////////////////////////////////////////////////////////////////////////////
//Pro-nav guidance
//applicable to mguidance:
//  						= 60 po-nav lateral  
//							= 06 pro-nav in pitch 
//							= 66 po-nav lateral and in pitch 
//return output: APNB(3x1) - m/s^2
//where:
//		alcomx=APNB(2)/grav, lateral acceleration command - g's
//		ancomx=-APNB(3)/grav, normal acceleration command - g's
//
//010301 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::guidance_pronav()
{
	//local variables
	Matrix APNB(3,1);
	Matrix GRAV_G(3,1);

	//localizing module-variables
	//input data
	double pronav_gain=hyper[81].real();
	double bias=hyper[98].real();
	//input from other modules
	double grav=round3[11].real();
	Matrix TBG=hyper[46].mat();
	double range_go=hyper[105].real();
	Matrix WOEB=hyper[107].vec();
	double closing_speed=hyper[108].real();
	double psisbx=hyper[110].real();
	double thtsbx=hyper[111].real();
	Matrix UTBB=hyper[113].vec();
	//-------------------------------------------------------------------------
	//gravity vector in geographic coordinates with bias
	GRAV_G.assign_loc(2,0,grav+bias);

	//required acceleration in body coordinates
	APNB=WOEB.skew_sym()*UTBB*(pronav_gain*closing_speed)-TBG*GRAV_G;
	
	return APNB;
}
//%%%///////////////////////////////////////////////////////////////////////////////
//Guidance to a line
//Against stationary waypoints or targets
//Waypoint are provided as: 'wp_lonx', 'wp_latx', 'wp_alt'
//applicable to mguidance:
//			= 30 line-guidance lateral 
//			= 03 line-guidance in pitch 
//			= 33 line-guidance lateral and in pitch
//return output:
//	 ALGV, acceleration demanded by line guidance in velocity coord. - m/s^2
//where:
//		alcomx=ALGV(2)/grav, lateral acceleration command - g's
//		ancomx=-ALGV(3)/grav, normal acceleration command - g's
//
//010405 Created by Peter H Zipfel
//010904 Upgraded to Waypoint guidance, PZi
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::guidance_line()
{
	//local variables
	Matrix ALGV(3,1);
	Matrix TFG(3,3);
	Matrix SWII(3,1);
	Matrix TOG(3,3);
	Matrix POLAR(3,1);
	Matrix VH(3,1);
	Matrix SH(3,1);
	double psiog(0);
	double thtog(0);
	double swbg1(0);
	double swbg2(0);
	double vbeo2(0);
	double vbeo3(0);
	double vbef2(0);
	double vbef3(0);
	double algv1(0);
	double algv2(0);
	double algv3(0);
	double vbeg1(0);
	double vbeg2(0);
	double dvbe(0);
	double rad_min(0);

	//local module-variables
	double wp_sltrange(0);
	double wp_grdrange(0);
	Matrix VBEO(3,1);
	Matrix VBEF(3,1);
	Matrix SWBG(3,1);
	double nl_gain(0);
	int wp_flag(0);

	//localizing module-variables
	//input data
	double line_gain=hyper[82].real();
	double nl_gain_fact=hyper[83].real();
	double decrement=hyper[84].real();
	double wp_lonx=hyper[85].real();
	double wp_latx=hyper[86].real();
	double wp_alt=hyper[87].real();
	double psifgx=hyper[88].real();
	double thtfgx=hyper[89].real();
	//input from other modules
	double time=round3[0].real(); 
	double grav=round3[11].real();
	Matrix TIG=round3[23].mat();
	double thtvgx=round3[29].real();
	Matrix VBEG=round3[32].vec();
	Matrix SBII=round3[35].vec();
	double philimx=hyper[56].real();
	//-------------------------------------------------------------------------
	//TM of LOA wrt geographic axes
	TFG=mat2tr(psifgx*RAD,thtfgx*RAD);

	//converting waypoint to inertial coordinates
	SWII=cadine(wp_lonx*RAD,wp_latx*RAD,wp_alt,time);

	//waypoint wrt hyper missile displacement in geographic coord (synthetic LOS)
	SWBG=TIG.trans()*(SWII-SBII);

	//building TM of LOS wrt geographic axes; also getting range-to-go to waypoint
	POLAR=SWBG.pol_from_cart();
	wp_sltrange=POLAR.get_loc(0,0);
	psiog=POLAR.get_loc(1,0);
	thtog=POLAR.get_loc(2,0);
	TOG=mat2tr(psiog,thtog);

	//ground range to waypoint (approximated by using hyper missile local-level plane)
	swbg1=SWBG.get_loc(0,0);
	swbg2=SWBG.get_loc(1,0);
	wp_grdrange=sqrt(swbg1*swbg1+swbg2*swbg2);

	//converting geographic hyper missile velocity to LOS and LOA coordinates
	VBEO=TOG*VBEG;
	vbeo2=VBEO.get_loc(1,0);
	vbeo3=VBEO.get_loc(2,0);

	VBEF=TFG*VBEG;
	vbef2=VBEF.get_loc(1,0);
	vbef3=VBEF.get_loc(2,0);

	//nonlinear gain
	nl_gain=nl_gain_fact*(1-exp(-wp_sltrange/decrement));

	//line guidance steering law
	algv1=grav*sin(thtvgx*RAD);
	algv2=line_gain*(-vbeo2+nl_gain*vbef2);
	algv3=line_gain*(-vbeo3+nl_gain*vbef3)-grav*cos(thtvgx*RAD);

	//packing accelerations int vector
	ALGV.assign_loc(0,0,algv1);
	ALGV.assign_loc(1,0,algv2);
	ALGV.assign_loc(2,0,algv3);

	//setting way point flag (if within 2 times turning radius): closing (+1) or fleeting (-1)
	dvbe=VBEG.absolute();
	rad_min=dvbe*dvbe/(grav*tan(philimx*RAD));
	if(wp_grdrange<2*rad_min)
	{
		//projection of displacement vector into horizontal plane, SH
		SH.assign_loc(0,0,swbg1);
		SH.assign_loc(1,0,swbg2);
		SH.assign_loc(2,0,0);
		//projection of velocity vector into horizontal plane, VH
		vbeg1=VBEG.get_loc(0,0);
		vbeg2=VBEG.get_loc(1,0);
		VH.assign_loc(0,0,vbeg1);
		VH.assign_loc(1,0,vbeg2);
		VH.assign_loc(2,0,0);
		//setting flag eihter to +1 or -1
		wp_flag=sign(VH^SH);
	}
	else
		wp_flag=0;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[91].gets(wp_sltrange);
	hyper[92].gets(nl_gain);
	hyper[93].gets_vec(VBEO);
	hyper[94].gets_vec(VBEF);
	hyper[95].gets(wp_grdrange);
	hyper[96].gets_vec(SWBG);
	hyper[97].gets(rad_min);
	hyper[99].gets(wp_flag);

	return ALGV;
}

//%%%///////////////////////////////////////////////////////////////////////////////
//Guidance to a point
//special case of line guidance
//Against stationary waypoints or targets
//Waypoint are provided as: 'wp_lonx', 'wp_latx', 'wp_alt'
//applicable to mguidance:
//			= 40 point-guidance lateral 
//			= 04 point-guidance in pitch 
//			= 44 point-guidance lateral and in pitch
//return output:
//	 APGV, acceleration demanded by point guidance in velocity coord. - m/s^2
//where:
//		alcomx=APGV(2)/grav, lateral acceleration command - g's
//		ancomx=-APGV(3)/grav, normal acceleration command - g's
//
//010816 Created by Peter H Zipfel
//010830 Upgraded to Waypoint guidance, PZi
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::guidance_point()
{
	//local variables
	Matrix APGV(3,1);
	Matrix TFG(3,3);
	Matrix SWII(3,1);
	Matrix TOG(3,3);
	Matrix POLAR(3,1);
	Matrix VH(3,1);
	Matrix SH(3,1);
	double psiog(0);
	double thtog(0);
	double vbeg1(0);
	double vbeg2(0);
	double dvbe(0);
	double swbg1(0);
	double swbg2(0);
	double vbeo2(0);
	double vbeo3(0);
	double apgv1(0);
	double apgv2(0);
	double apgv3(0);

	//local module-variables
	double wp_sltrange(0);
	double wp_grdrange(0);
	double rad_min(0);
	int wp_flag(0);
	Matrix VBEO(3,1);
	Matrix SWBG(3,1);

	//localizing module-variables
	//input data
	double wp_lonx=hyper[85].real();
	double wp_latx=hyper[86].real();
	double wp_alt=hyper[87].real();
	double point_gain=hyper[90].real();
	//input from other modules
	double time=round3[0].real(); 
	double grav=round3[11].real();
	Matrix TIG=round3[23].mat();
	double thtvgx=round3[29].real();
	Matrix VBEG=round3[32].vec();
	Matrix SBII=round3[35].vec();
	double philimx=hyper[56].real();
	if (time > 54){
		double dummy=0;
	}
	//-------------------------------------------------------------------------
	//converting waypoint to inertial coordinates
	SWII=cadine(wp_lonx*RAD,wp_latx*RAD,wp_alt,time);

	//waypoint wrt hyper missile displacement in geographic coord (synthetic LOS)
	SWBG=TIG.trans()*(SWII-SBII);

	//building TM of LOS wrt geographic axes; also getting range-to-go to waypoint
	POLAR=SWBG.pol_from_cart();
	wp_sltrange=POLAR.get_loc(0,0);
	psiog=POLAR.get_loc(1,0);
	thtog=POLAR.get_loc(2,0);
	TOG=mat2tr(psiog,thtog);

	//ground range to waypoint (approximated by using hyper missile local-level plane)
	swbg1=SWBG.get_loc(0,0);
	swbg2=SWBG.get_loc(1,0);
	wp_grdrange=sqrt(swbg1*swbg1+swbg2*swbg2);

	//converting geographic hyper missile velocity to LOS and LOA coordinates
	VBEO=TOG*VBEG;
	vbeo2=VBEO.get_loc(1,0);
	vbeo3=VBEO.get_loc(2,0);

	//point guidance steering law
	apgv1=grav*sin(thtvgx*RAD);
	apgv2=point_gain*(-vbeo2);
	apgv3=point_gain*(-vbeo3)-grav*cos(thtvgx*RAD);

	//packing accelerations into vector
	APGV.assign_loc(0,0,apgv1);
	APGV.assign_loc(1,0,apgv2);
	APGV.assign_loc(2,0,apgv3);

	//setting way point flag (if within 2 times turning radius): closing (+1) or fleeting (-1)
	dvbe=VBEG.absolute();
	rad_min=dvbe*dvbe/(grav*tan(philimx*RAD));
	if(wp_grdrange<2*rad_min)
	{
		//projection of displacement vector into horizontal plane, SH
		SH.assign_loc(0,0,swbg1);
		SH.assign_loc(1,0,swbg2);
		SH.assign_loc(2,0,0);
		//projection of velocity vector into horizontal plane, VH
		vbeg1=VBEG.get_loc(0,0);
		vbeg2=VBEG.get_loc(1,0);
		VH.assign_loc(0,0,vbeg1);
		VH.assign_loc(1,0,vbeg2);
		VH.assign_loc(2,0,0);
		//setting flag eihter to +1 or -1
		wp_flag=sign(VH^SH);
	}
	else
		wp_flag=0;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[91].gets(wp_sltrange);
	hyper[93].gets_vec(VBEO);
	hyper[95].gets(wp_grdrange);
	hyper[96].gets_vec(SWBG);
	hyper[97].gets(rad_min);
	hyper[99].gets(wp_flag);

	return APGV;
}
//%%%///////////////////////////////////////////////////////////////////////////////
//Guidance on an arc through a waypoint (horizontal only)
//Waypoints are provided as: 'wp_lonx', 'wp_latx', 'wp_alt'
//applicable to:
//			mguidance = 70 lateral arc-guidance 
//return output:
//	 phicx = commanded bank angle - deg
//
//010823 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Hyper::guidance_arc()
{
	//local variables
	Matrix SWII(3,1);
	Matrix POLAR(3,1);
	Matrix TBV(3,3);
	Matrix FSPB(3,1);
	Matrix VH(3,1);
	Matrix SH(3,1);
	Matrix UV(3,1);
	Matrix ZZ(3,1);
	double dwbh(0);
	double vbeg1(0);
	double vbeg2(0);
	double swbg1(0);
	double swbg2(0);
	double psiwvx(0);
	double fspb3(0);
	double num(0);
	double denom(0);
	double argument(0);
	double phicx(0);
	double alpha(0);
	double phimv(0);
	double rad_dynamic(0);
	double rad_geometric(0);

	//local module-variables
	Matrix SWBG(3,1);
	int wp_flag(0);
	double wp_grdrange(0);
	double rad_min(0);

	//localizing module-variables
	//input data
	double wp_lonx=hyper[85].real();
	double wp_latx=hyper[86].real();
	double wp_alt=hyper[87].real();
	//input from other modules
	double time=round3[0].real();
	Matrix FSPV=round3[10].vec();
	double grav=round3[11].real();
	Matrix TIG=round3[23].mat();
	double dvbe=round3[25].real();
	double psivgx=round3[28].real();
	Matrix VBEG=round3[32].vec();
	Matrix SBII=round3[35].vec();
	double alphax=hyper[51].real();
	double phimvx=hyper[52].real();
	double philimx=hyper[56].real();
	if (time > 49)
	{
		double test=0;
	}
	//-------------------------------------------------------------------------
	//converting waypoint coordinates
	SWII= cadine(wp_lonx*RAD,wp_latx*RAD,wp_alt,time);

	//displacement of waypoint wrt hyper missile in geographic coord
	SWBG=TIG.trans()*(SWII-SBII);

	//projection of displacement vector into horizontal plane, SH
	swbg1=SWBG.get_loc(0,0);
	swbg2=SWBG.get_loc(1,0);
	SH.assign_loc(0,0,swbg1);
	SH.assign_loc(1,0,swbg2);
	SH.assign_loc(2,0,0);

	//horizontal ground distance based on hyper missile geographic coordinates
	dwbh=sqrt(swbg1*swbg1+swbg2*swbg2);

	//calculating azimuth angle of waypoint LOS wrt velocity vector, psiwvx
	//projection of velocity vector into horizontal plane, VH
	vbeg1=VBEG.get_loc(0,0);
	vbeg2=VBEG.get_loc(1,0);
	VH.assign_loc(0,0,vbeg1);
	VH.assign_loc(1,0,vbeg2);
	VH.assign_loc(2,0,0);

	//vector normal to arc plane, UV
	UV=VH.skew_sym()*SH;
	//steering angle, psiwvx 
	psiwvx=DEG*angle(VH,SH);
	//steering angle with proper sign
	ZZ.assign_loc(2,0,1);
	psiwvx=psiwvx*sign(UV^ZZ);

	//tranforming specific force to body coordinates and picking third component
	alpha=alphax*RAD;
	phimv=phimvx*RAD;
	TBV=cadtbv(phimv,alpha);
	FSPB=TBV*FSPV;
	fspb3=FSPB.get_loc(2,0);

	//selecting guidance mode
	if(fabs(psiwvx)<90)
		//guiding on the arc through the waypoint
	{
		num=-2*dvbe*dvbe*sin(psiwvx*RAD);
		denom=fspb3*dwbh;
		if(denom!=0) argument=num/denom;
		if(fabs(asin(argument))<philimx*RAD)
			phicx=DEG*asin(argument);
		else
			phicx=philimx*sign(argument);
	}
	else
	{
		//making a minimum turn
		phicx=philimx*sign(psiwvx);
	}
	//diagnostic: radii
	if(phimvx!=0) rad_dynamic=fabs(dvbe*dvbe/(fspb3*sin(phimvx*RAD)));
	if(psiwvx!=0) rad_geometric=fabs(dwbh/(2*sin(psiwvx*RAD)));
		
	//setting way point flag (if within 2 times turning radius): closing (+1) or fleeting (-1)
	rad_min=dvbe*dvbe/(grav*tan(philimx*RAD));
	//if(dwbh<rad_min)
	if(dwbh<0.2*rad_min) 
		wp_flag=sign(VH^SH);
	else
		wp_flag=0;

	//ground range to waypoint
	wp_grdrange=dwbh;

	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	hyper[96].gets_vec(SWBG);
	//diagnostics
	hyper[95].gets(wp_grdrange); 
	hyper[97].gets(rad_min);
	hyper[99].gets(wp_flag);

	return phicx;
}

//$$$/////////////////////////////////////////////////////////////////////////////
//Definition of control module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[40-79]
//		
//001228 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Hyper::def_control()
{
	//definition of module-variables
	hyper[40].init("mcontrol","int",0,"Mode switch - ND","control","data","scrn");
	hyper[41].init("psivgcx",0,"Commanded heading angle - deg","control","data","plot");
	hyper[42].init("thtvgcx",0,"Commanded flight path angle - deg","control","data","plot");
	hyper[43].init("alphacx",0,"Commanded angle of attack - deg","control","data","");
	hyper[44].init("phimvcx",0,"Commanded bank angle - deg","control","data","");
	hyper[45].init("TBV",0,0,0,0,0,0,0,0,0,"TM of body wrt velocity coord. - ND","control","out","");
	hyper[46].init("TBG",0,0,0,0,0,0,0,0,0,"TM of body wrt geog. coord. - ND","control","out","");
	hyper[47].init("gain_thtvg",0,"Flight path angle hold control gain - g/deg","control","data","");
	hyper[48].init("gain_psivg",0,"Heading angle hold control gain - ND","control","data","");
	hyper[49].init("anx",0,"Normal load factor - g","control","diag","scrn,plot");
	hyper[50].init("avx",0,"Vertical load factor - g","control","diag","scrn,plot");
	hyper[51].init("alphax",0,"Angle of attack - deg","control","out","scrn,plot");
	hyper[52].init("phimvx",0,"Bank angle - deg","control","out","scrn,plot");
	hyper[53].init("phicx",0,"Bank angle command- deg","control","data","scrn,plot");
	hyper[54].init("phix",0,"Bank angle state - deg","control","state","plot");
	hyper[55].init("phixd",0,"Bank angle state derivative - deg/sec","control","state","");
	hyper[56].init("philimx",0,"Bank angle command limiter - deg","control","data","");
	hyper[57].init("tphi",0,"Time constant of bank angle response - sec","control","data","");
	hyper[58].init("anposlimx",0,"Positive load factor limiter - g's","control","data","");
	hyper[59].init("anneglimx",0,"Negative load factor limiter - g's","control","data","");
	hyper[60].init("gacp",0,"Root locus gain of accel loop - rad/s^2","control","data","");
	hyper[61].init("ta",0,"Ratio of prop/integral gains. If>0, P-I engaged","control","data","");
	hyper[62].init("xi",0,"Integral feedback - rad/s","control","state","");
	hyper[63].init("xid",0,"Integral feedback derivative - rad/s^2","control","state","");
	hyper[64].init("qq",0,"Pitch rate - rad/s","control","diag","plot");
	hyper[65].init("tip",0,"Incidence lag time constant - s","control","diag","plot");
	hyper[66].init("alp",0,"Angle of attack - rad","control","state","");
	hyper[67].init("alpd",0,"Angle of attack derivative - rad/s","control","state","");
	hyper[68].init("alpposlimx",0,"Angle of attack positive limiter - deg","control","data","");
	hyper[69].init("alpneglimx",0,"Angle of attack negative limiter - deg","control","data","");
	hyper[70].init("ancomx",0,"Load factor command - g's","control","data","scrn,plot");
	hyper[71].init("alcomx",0,"Lateral acceleration command - g's","control","data","scrn,plot");
	hyper[72].init("allimx",0,"Lateral acceleration limiter - g's","control","data","");
	hyper[73].init("gcp",0,"Lateral roll gain - rad","control","data","");
	hyper[74].init("alx",0,"Lateral acceleration - g's","control","diag","plot");
	hyper[75].init("altdlim",0,"Altitude rate limiter - m/s","control","data","");
	hyper[76].init("gh",0,"Altitude gain - g/m","control","data","");
	hyper[77].init("gv",0,"Altitude rate gain - g/(m/s)","control","data","");
	hyper[78].init("altd",0,"Altitude rate  - m/s","control","diag","plot");
	hyper[79].init("altcom",0,"Altitude command  - m","control","data","plot");
}
//$$$////////////////////////////////////////////////////////////////////////// 
//Control module
//Member function of class 'Hyper' 
//
//mcontrol = 00: No control, phimv=0, alpha=0
//			 01: Flight path angle control, no heading control, thtvgcx
//			 10: Heading and alpha control, psivgcx, alphacx
//			 11: Heading and flight path angle control, thtvgcx and psivgcx
//			 03: Bank angle controller, phicx
//			 04: Acceleration control in load-factor plane, ancomx
//			 40: Acceleration control in lateral (horizontal) plane, alcomx
//			 44: Acceleration control in lateral and load-factor plane alcomx, ancomx -> used in HYPER5
//			 06: Altitude hold control with inner acceleration autopilot, altcom
//			 16: Heading and altitude hold controller, psivgcx, altcom -> used in HYPER5			 			  
//			 46: Lateral acceleration and altitude controller, alcomx, altcom
//			 36: Bank angle control and altitude hold (used with mguidance=70), phicx, altcom -> used in HYPER5
//
//000725 Created by Michael Horvath
//001228 Upgraded to module-variable arrays, PZi
//010126 Added acceleration and altitude controllers, PZi
///////////////////////////////////////////////////////////////////////////////	
void Hyper::control(double int_step)
{
	//local module-variables
	double phimvx(0);
	double alphax(0);
	Matrix TBV(3,3);
	Matrix TBG(3,3);
	Matrix TVG(3,3);

	//localizing module-variables
	//input data
	int mcontrol=hyper[40].integer();
	double psivgcx=hyper[41].real();
	double thtvgcx=hyper[42].real();
	double alphacx=hyper[43].real();
	double ancomx=hyper[70].real();
	double alcomx=hyper[71].real();
	double altcom=hyper[79].real();
	//restore saved values
	double phicx=hyper[53].real();
	//input from other modules
	Matrix TGV=round3[22].mat();
	//-------------------------------------------------------------------------
	//No control
	if (mcontrol==0)
	{
		phimvx=0;
		alphax=0;
	}
	//Flight path angle control
	if (mcontrol==1)
	{
		phimvx=0;
		alphax=control_flightpath(thtvgcx,phimvx);
	}
	//Heading and alpha control
	if (mcontrol==10)
	{
		phicx=control_heading(psivgcx);
		phimvx=control_bank(phicx,int_step);
		alphax=alphacx;
	}
	//heading and flight path angle control
	if (mcontrol==11)
	{
		phicx=control_heading(psivgcx);
		phimvx=control_bank(phicx,int_step);
		alphax=control_flightpath(thtvgcx,phimvx);
	}
	//bank angle and alpha control
	if (mcontrol==3)
	{
		phimvx=control_bank(phicx,int_step);
		alphax=alphacx;
	}
	//load factor control
	if (mcontrol==4)
	{
		alphax=control_load(ancomx,int_step);
	}
	//lateral acceleration control
	if (mcontrol==40)
	{
		phicx=control_lateral(alcomx);
		phimvx=control_bank(phicx,int_step);
	}
	//acceleration control in both planes
	if (mcontrol==44)
	{
		phicx=control_lateral(alcomx);
		phimvx=control_bank(phicx,int_step);
		alphax=control_load(ancomx,int_step);
	}

	//altitude control
	if (mcontrol==6)
	{
		ancomx=control_altitude(altcom,phimvx);
		alphax=control_load(ancomx,int_step);
	}
	//heading and altitude
	if (mcontrol==16)
	{
		phicx=control_heading(psivgcx);
		phimvx=control_bank(phicx,int_step);

		ancomx=control_altitude(altcom,phimvx);
		alphax=control_load(ancomx,int_step);
	}
	//lateral acceleration and altitude control
	if (mcontrol==46)
	{
		phicx=control_lateral(alcomx);
		phimvx=control_bank(phicx,int_step);

		ancomx=control_altitude(altcom,phimvx);
		alphax=control_load(ancomx,int_step);
	}
	//lateral bank control and altitude control
	if (mcontrol==36)
	{
		phimvx=control_bank(phicx,int_step);

		ancomx=control_altitude(altcom,phimvx);
		alphax=control_load(ancomx,int_step);
	}
	//calculating TBV and direction cosine matrix TBG;
	TBV=cadtbv(phimvx*RAD,alphax*RAD);
	TVG=TGV.trans();
	TBG=TBV*TVG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving variables
	hyper[53].gets(phicx);
	//output to other modules
	hyper[45].gets_mat(TBV);
	hyper[46].gets_mat(TBG);
	hyper[51].gets(alphax);
	hyper[52].gets(phimvx);
	hyper[70].gets(ancomx);
}

//%%%/////////////////////////////////////////////////////////////////////////////
//Heading angle control
//
//return output: phimvx, bank angle - deg 
//
//000725 Created by Michael Horvath
//001228 Upgraded to module-variable arrays, PZi
//$$///////////////////////////////////////////////////////////////////////////////

double Hyper::control_heading(double psivgcx)
{
	//local variables
	double phimvx(0);
	double psivgx_comp(0);
	double sign_psivgx(0);
	
	//localizing module-variables
	//input data
	double gain_psivg=hyper[48].real();
	//input from other modules
	double psivgx=round3[28].real();
	//-------------------------------------------------------------------------
	//elininating the singulatiry at psivgx=+-180 deg by giving special
	//treatment to the heading control of +-45 deg from south
	if(fabs(psivgcx)<=135)
		psivgx_comp=psivgx;
	else
	{
		if(psivgx*psivgcx>=0)
			psivgx_comp=psivgx;
		else
		{
			if(psivgx>=0)sign_psivgx=1;
			else sign_psivgx=-1;
			psivgx_comp=360-psivgx*sign_psivgx;
		}
	}
	phimvx=gain_psivg*(psivgcx-psivgx_comp);
	return phimvx;
}

//%%%///////////////////////////////////////////////////////////////////////////////
//Flight path angle control
//
//return output: alphax, angle of attack - deg
//
//000725 Created by Michael Horvath
//001228 Upgraded to module-variable arrays, PZi
//$$///////////////////////////////////////////////////////////////////////////////

double Hyper::control_flightpath(double thtvgcx,double phimvx)
{

	//local module-variables
	double anx(0);
	double avx(0);
	double alphax(0);
	
	//localizing module-variables
	//input data
	double gain_thtvg=hyper[47].real();
	double alpposlimx=hyper[68].real();
	double alpneglimx=hyper[69].real();
	//input from other modules
	double pdynmc=round3[13].real();
	double thtvg=round3[18].real();
	double grav=round3[11].real();
	double mass=hyper[15].real();
	double area=hyper[33].real();
	double cla=hyper[34].real();
	//-------------------------------------------------------------------------
	//vertical and normal loadfactors
	avx=gain_thtvg*(thtvgcx*RAD-thtvg);
	anx=((avx)/(cos(phimvx*RAD)));
	alphax=((anx*mass*grav)/(pdynmc*area*cla));

	//limiting angle of attack
	if(alphax>alpposlimx) alphax=alpposlimx;
	if(alphax<alpneglimx) alphax=alpneglimx;

	//loading module-variables
	hyper[49].gets(anx);
	hyper[50].gets(avx);

	return alphax;
}
//%%%///////////////////////////////////////////////////////////////////////////////
//Bank angle controller
//
//return output: phix, bank angle - deg
//
//010126 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////////

double Hyper::control_bank(double phicx,double int_step)
{
	//local variables
	double phixd_new(0);
	
	//localizing module-variables
	//input data
	double philimx=hyper[56].real();
	double tphi=hyper[57].real();
	//state variables
	double phix=hyper[54].real();
	double phixd=hyper[55].real();
	//-------------------------------------------------------------------------	
	//limiting bank angle command
	if(phicx>philimx) phicx=philimx;
	if(phicx<-philimx) phicx=-philimx;

	//bank angle lag
	phixd_new=(phicx-phix)/tphi;
	phix=integrate(phixd_new,phixd,phix,int_step);
	phixd=phixd_new;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	hyper[54].gets(phix);
	hyper[55].gets(phixd);

	return phix;
}
//%%%///////////////////////////////////////////////////////////////////////////////
//Load factor controller
//
//return output: alpx, angle of attack - deg
//
//010129 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////////

double Hyper::control_load(double ancomx,double int_step)
{
	//local variables
	Matrix FSPB(3,1);
	Matrix TBV(3,3);
	double alpha(0);
	double phimv(0);
	double fspb3(0);
	double eanx(0);
	double gr(0);
	double gi(0);
	double xid_new(0);
	double alpd_new(0);
	double alpx(0);

	//local module-variables
	double anx(0);
	double tip(0);
	double qq(0);
	
	//localizing module-variables
	//input data
	double anposlimx=hyper[58].real();
	double anneglimx=hyper[59].real();
	double phimvx=hyper[52].real();
	double gacp=hyper[60].real();
	double ta=hyper[61].real();
	double alphax=hyper[51].real();
	double alpposlimx=hyper[68].real();
	double alpneglimx=hyper[69].real();
	//input from other modules
	Matrix FSPV=round3[10].vec();
	double grav=round3[11].real();
	//state variables
	double xi=hyper[62].real();
	double xid=hyper[63].real();
	double alp=hyper[66].real();
	double alpd=hyper[67].real();
	//input from other modules
	double mass=hyper[15].real();
	double dvbe=round3[25].real();
	double pdynmc=round3[13].real();
	double thrust=hyper[26].real();
	double area=hyper[33].real();
	double cla=hyper[34].real();
	//-------------------------------------------------------------------------
	//tranforming specific force to body coordinates
	alpha=alphax*RAD;
	phimv=phimvx*RAD;
	TBV=cadtbv(phimv,alpha);
	FSPB=TBV*FSPV;

	//limiting load factor command
	if(ancomx>anposlimx) ancomx=anposlimx;
	if(ancomx<anneglimx) ancomx=anneglimx;

	//load factor feedback
	fspb3=FSPB.get_loc(2,0);
	anx=-fspb3/grav;

	//error signal
	eanx=ancomx-anx;

	//incidence lag time constant
	tip=dvbe*mass/(pdynmc*area*cla/RAD+thrust);

	//integral path
	if(ta>0)
	{
		gr=gacp*tip/dvbe;
		gi=gr/ta;
		xid_new=gi*eanx;
		xi=integrate(xid_new,xid,xi,int_step);
		xid=xid_new;
	}else xi=0;

	//proportional path with integral signal(if present)
	qq=gr*eanx+xi;

	//incidence lag dynamics
	alpd_new=qq-alp/tip;
	alp=integrate(alpd_new,alpd,alp,int_step);
	alpd=alpd_new;

	alpx=alp*DEG;

	//limiting angle of attack
	if(alpx>alpposlimx) alpx=alpposlimx;
	if(alpx<alpneglimx) alpx=alpneglimx;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	hyper[62].gets(xi);
	hyper[63].gets(xid);
	hyper[66].gets(alp);
	hyper[67].gets(alpd);
	//diagnostics
	hyper[49].gets(anx);
	hyper[64].gets(qq);
	hyper[65].gets(tip);

	return alpx;
}
//%%%///////////////////////////////////////////////////////////////////////////////
//Lateral acceleration controller
//
//return output: phicx, bank command - deg
//
//010130 Created by Peter H Zipfel
//240117 Modified for HYPER5, PZi
///////////////////////////////////////////////////////////////////////////////////

double Hyper::control_lateral(double alcomx)
{
	//local variables
	Matrix FSPB(3,1);
	Matrix TBV(3,3);
	double alpha(0);
	double phimv(0);
	double fspb3(0);
	double anx(0);
	double sign(0);
	double phic(0);
	double phicx(0);
	double fspv2(0);

	//local module-variables
	double alx(0);
	
	//localizing module-variables
	//input data
	double allimx=hyper[72].real();
	double gcp=hyper[73].real();
	//input from other modules
	Matrix FSPV=round3[10].vec();
	double grav=round3[11].real();
	double phimvx=hyper[52].real();
	double alphax=hyper[51].real();
	//-------------------------------------------------------------------------
	//tranforming specific force to body coordinates
	alpha=alphax*RAD;
	phimv=phimvx*RAD;
	TBV=cadtbv(phimv,alpha);
	FSPB=TBV*FSPV;
	//normal load factor
	fspb3=FSPB.get_loc(2,0);
	anx=-fspb3/grav;

	//limiting lateral load factor command
	if(alcomx>allimx) alcomx=allimx;
	if(alcomx<-allimx) alcomx=-allimx;

	phic=atan2(alcomx,anx);
	phicx=phic*DEG;

	//diagnostic: lateral acceleration achieved
	fspv2=FSPV.get_loc(1,0);
	alx=fspv2/grav;

	//-------------------------------------------------------------------------
	//loading module-variables
	hyper[74].gets(alx);

	return phicx;
}

//%%%///////////////////////////////////////////////////////////////////////////////
//Altitude controller
//
//return output: ancomx, load factor command - g's
//
//010131 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////////

double Hyper::control_altitude(double altcom,double phimvx)
{
	//local variables
	double ealt(0);
	double ancomx(0);

	//local module-variables
	double altd(0);
	
	//localizing module-variables
	//input data
	double anposlimx=hyper[58].real();
	double anneglimx=hyper[59].real();
	double altdlim=hyper[75].real();
	double gh=hyper[76].real();
	double gv=hyper[77].real();
	//input from other modules
	double alt=round3[21].real();
	double grav=round3[11].real();
	Matrix VBEG=round3[32].vec();
	//-------------------------------------------------------------------------
	//altitude error
	ealt=gh*(altcom-alt);

	//limiting altitude rate
	if(ealt>altdlim) ealt=altdlim;
	if(ealt<-altdlim) ealt=-altdlim;

	//altitude rate feedback
	altd=-VBEG.get_loc(2,0);

	//load factor command
	ancomx=(gv*(ealt-altd)/grav+1)*(1/cos(phimvx*RAD));

	//limiting load factor command
	if(ancomx>anposlimx) ancomx=anposlimx;
	if(ancomx<anneglimx) ancomx=anneglimx;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnsotics
	hyper[78].gets(altd);

	return ancomx;
}
//$$$/////////////////////////////////////////////////////////////////////////////
//Definition of intercept module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[120-129]
//		
//010328 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Hyper::def_intercept()
{
	//definition of module-variables
	hyper[121].init("write","int",1,"True flag for writing miss to console - ND","intercept","save","");
	hyper[122].init("miss",0,"Miss distance - m","intercept","diag","");
	hyper[123].init("hit_time",0,"Intercept time - s","intercept","diag","");
	hyper[124].init("MISS_G",0,0,0,"Miss vector in geog. coord. - m","intercept","diag","");
	hyper[125].init("time_m",0,"Previous time - s","intercept","save","");
	hyper[126].init("SBTGM",0,0,0,"Previous displacment vector. - m","intercept","save","");
	hyper[127].init("STMEG",0,0,0,"Previous taget displacment vector. - m","intercept","save","");
	hyper[128].init("SBMEG",0,0,0,"Previous hyper missile displacment vector. - m","intercept","save","");
	hyper[129].init("halt","int",0," =0 false, =1 stops run","intercept","data","");
}
//$$$//////////////////////////////////////////////////////////////////////////
//Intercept module
//Member function of class 'Hyper'
//Determining closest approach of hyper missile and target points
//
//Parameter Input: 'vehicle_slot' is current 'Hyper' object
//Input from module-variable array: 'targ_com_slot' target being attacked, determined in 'seeker' module
//
//Console output: miss distance and associated parameters written to console
//
//010328 Created by Peter H Zipfel
//231905 Additional info for HYPER5, PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::intercept(Packet *combus,int vehicle_slot,double int_step,const char *title)
{
	//local variables
	double tau(0);
	Matrix SBTG(3,1);
	Matrix SBBMG(3,1);
	double swbg1(0);
	double swbg2(0);
	double dwbh(0);

	//local module-variables
	double hit_time(0);
	Matrix MISS_G(3,1);
	double miss(0);

	//localizing module-variables
	int halt=hyper[129].integer();	
	//input from other modules
	double time=round3[0].real();
	double dvbe=round3[25].real();
	double alt=round3[21].real();
	double psivgx=round3[28].real();
	double thtvgx=round3[29].real();
	double ground_range=round3[38].real();
	double phimvx=hyper[52].real();
	Matrix SBEG=round3[31].vec();
	int mguidance=hyper[80].integer();
	double wp_lonx=hyper[85].real();
	double wp_latx=hyper[86].real();
	double wp_alt=hyper[87].real();
	Matrix SWBG=hyper[96].vec();
	int wp_flag=hyper[99].integer();
	int mseeker=hyper[100].integer();
	double range_go=hyper[105].real();
	Matrix STBG=hyper[106].vec();
	double closing_speed=hyper[108].real();
	int targ_com_slot=hyper[112].integer();
	//restore saved values
	int write=hyper[121].integer();
	double time_m=hyper[125].real();
	Matrix SBTGM=hyper[126].vec();
	Matrix STMEG=hyper[127].vec();
	Matrix SBMEG=hyper[128].vec();
	//-------------------------------------------------------------------------
	//stopping run with 'halt'
	if(halt&&write)
	{
		write=0;
		//getting hyper missile #
		string id_missl=combus[vehicle_slot].get_id();
		cout << "\n" << " *** Run halted of Missile_" << id_missl << "  Ground range = " << ground_range << " m  Time = " << time << " sec ***\n";
		combus[vehicle_slot].set_status(0);
		wp_flag=0;
	}
	//Ground impact
	if((alt<=0)&&write)
	{
		write=0;

		//getting hyper missile #
		string id_missl=combus[vehicle_slot].get_id();

		//writing miss information to console
//		cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
		cout<<"\n"<<" *** Ground impact of Missile_"<<id_missl<<"   Time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivgx<<" deg      gamma = "<<thtvgx<<" deg\n\n";    

		//declaring hyper missile 'dead'
		combus[vehicle_slot].set_status(0);
	}
	//Waypoint hoizontal miss distance 
	if(mguidance==70||mguidance==40||mguidance==30||mguidance==44)
	{
		if(wp_flag==-1)
		{

			swbg1=SWBG.get_loc(0,0);
			swbg2=SWBG.get_loc(1,0);
			dwbh=sqrt(swbg1*swbg1+swbg2*swbg2);

			//getting hyper missile #
			string id_missl=combus[vehicle_slot].get_id();

			//writing information to console
			cout<<"\n"<<" *** Missile "<<id_missl<<" overflies waypoint at longitude = "
				<<wp_lonx<<" deg, latitude = "<<wp_latx<<" deg at time = "<<time<<" sec *** \n";
			cout<<"      SWBG-horizontal miss distance  = "<<dwbh<<" m north = "<<SWBG.get_loc(0,0)
				<<" m  east = "<<SWBG.get_loc(1,0)<<" m\n";   
			cout<<"      bank angle  = "<<phimvx<<" deg heading = "<<psivgx
				<<" deg  flight path angle = "<<thtvgx<<" deg\n";   
		}
	}
	//Terminal line/line or point/line guidance
	if(mguidance==33||mguidance==43)
	{
		//Line guidance: simple miss distance calculation at last integration, no interpolation
		if((alt<=wp_alt)&&write)
		{
			write=0;
			
			miss=SWBG.absolute();

			//getting hyper missile #
			string id_missl=combus[vehicle_slot].get_id();

			//writing miss information to console
	//		cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
			cout<<"\n"<<" *** Impact of Missile_"<<id_missl<<" on waypoint coord.: longitude = "<<wp_lonx<<" deg, latitude = "
							<<wp_latx<<" deg, altitude = "<<wp_alt<<" m\n";
			cout<<"      miss distance = "<<miss<<" m    intercept time = "<<time<<" sec\n";
			cout<<"      north = "<<SWBG.get_loc(0,0)<<" m      east = "<<SWBG.get_loc(1,0)
							<<" m        down = "<<SWBG.get_loc(2,0)<<" m\n";
			cout<<"      speed = "<<dvbe<<" m/s heading = "<<psivgx<<" deg     gamma = "<<thtvgx<<" deg\n\n";    

			//declaring hyper missile 'dead'
			combus[vehicle_slot].set_status(0);
		}
	}//end of terminal line/line or point/line guidance

	//Seeker/pronav
	if(mseeker==3)
	{
		//entering sphere of target influence of 100m 
		if(range_go<1000)
		{		
			Variable *data_t;
			Matrix STEG(3,1);
			Matrix SBBMG(3,1);
			Matrix STTMG(3,1);
			
			//get target location
			data_t=combus[targ_com_slot].get_data();
			STEG=data_t[8].vec();
			
			//Intercept (closing speed becomes negative)
			//Miss is closest distance between hyper missile and target points; obtained by linear interpolation
			//between integration steps
			SBTG=STBG*(-1);
			if((closing_speed<0)&&write)
			{
				write=0;

				SBBMG=SBEG-SBMEG;
				STTMG=STEG-STMEG;

				//intercept time at point of closest approach
				hit_time=time_m-int_step*((SBBMG-STTMG)^SBTGM)/(SBBMG^SBBMG);

				//miss distance vector in geographic coordinates
				tau=hit_time-time_m;
				MISS_G=(SBBMG-STTMG)*(tau/int_step)+SBTGM;
				miss=MISS_G.absolute();

				//getting hyper missile # and target #
				string id_targ=combus[targ_com_slot].get_id();
				string id_missl=combus[vehicle_slot].get_id();

				//writing miss information to console
				cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
				cout<<"\n"<<" *** Intercept of Missile_"<<id_missl<<" and Target_"<<id_targ<<" ***\n";
				cout<<"      miss distance = "<<miss<<" m    intercept time = "<<hit_time<<" sec\n";
				cout<<"      north = "<<MISS_G.get_loc(0,0)<<" m      east = "<<MISS_G.get_loc(1,0)
								<<" m        down = "<<MISS_G.get_loc(2,0)<<" m\n";
				cout<<"      speed = "<<dvbe<<" m/s heading = "<<psivgx<<" deg     gamma = "<<thtvgx<<" deg\n\n";    
				
				//declaring hyper missile 'dead (0)'and target 'hit (-1)'
				combus[vehicle_slot].set_status(0);
			}
			//save from previous cycle
			SBTGM=SBTG;
			STMEG=STEG;
			SBMEG=SBEG;
			time_m=time;
		}
	}//end of seeker/pronav
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving variables
	hyper[121].gets(write);
	hyper[125].gets(time_m);
	hyper[126].gets_vec(SBTGM);
	hyper[127].gets_vec(STMEG);
	hyper[128].gets_vec(SBMEG);
	//diagnostics
	hyper[122].gets(miss);
	hyper[123].gets(hit_time);
	hyper[124].gets_vec(MISS_G);
}
