///////////////////////////////////////////////////////////////////////////////
//FILE: 'intercept.cpp'
//Contains the 'intercept' module of class 'Missile'
//
//030712 Created by Peter H Zipfel
//060508 Modified for SWEEP++, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of intercept module-variables
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[650-699]
//
//
// Trajectory termination code	'lconv'=0 initialized
//										1 undefined
//										2 trajectory terminated with miss calculation
//										3 trajectory terminated at ground impact
//										4 trajectory stopped when termination code 'trcond' is set (and if 'stop=1')
//										5 trajectory halted when 'halt' is set in 'input.asc' ('halt=1')
//		
//030714 Created by Peter H Zipfel
//060509 Modified for SWEEP++, PZi
///////////////////////////////////////////////////////////////////////////////

void Missile::def_intercept()
{
	//definition of module-variables
	missile[650].init("mterm","int",0,"=0:miss magnitude; =1:in I-plane; =2:w/angle input - ND","intercept","data","");
	missile[651].init("write","int",true,"True flag for writing miss to console - ND","intercept","init","");
	missile[652].init("miss",0,"Miss distance - m","intercept","diag","plot");
	missile[653].init("hit_time",0,"Intercept time - s","intercept","diag","");
	missile[654].init("MISS",0,0,0,"Miss vector in L (mterm=0) or P (mterm>=1) coor - m","intercept","diag","plot");
	missile[655].init("time_m",0,"Previous time - s","intercept","save","");
	missile[656].init("SBTLM",0,0,0,"Previous displacment vector. - m","intercept","save","");
	missile[657].init("STMEL",0,0,0,"Previous aircraft taget displacment vector. - m","intercept","save","");
	missile[658].init("SBMEL",0,0,0,"Previous missile displacment vector. - m","intercept","save","");
	missile[659].init("mode","int",0,"Mode flags |mseek|mguid|maut|mprop|  - ND","intercept","diag","scrn");
	missile[661].init("psiptx",0,"Yaw angle of intercept plane - deg","intercept","/diag/data","plot");
	missile[662].init("thtptx",0,"Pitch angle of intercept plane - deg","intercept","diag/data","plot");
	missile[663].init("critmax",200,"Critish max value for miss - m","intercept","diag/data","plot");
}
///////////////////////////////////////////////////////////////////////////////
//Intercept module
//Member function of class 'Missile'
//Determining closest approach of missile and target points
//
//Parameter Input: 'vehicle_slot' is current 'Missile' object
//Input from module-variable array: 'tgt_com_slot' target being attacked, determined in 'seeker' module
//
// mterm = 0 miss of closest approach
//		   1 miss in intercept plane, that is given by calculated aspect angles
//         2 miss in intercept plane, but aspect angles are provided in 'input.asc'
//  if(mterm>=1) MISS vector is coordinated in intercept plane: MISS1 down, MISS2 right 
//
//Parameter Input: 'vehicle_slot' is current 'Missile' object
//Input from module-variable array: 'tgt_com_slot' target being attacked, determined in 'sensor' module
//
//* Intercept plane: the plane is normal to the differential  
//  velocity vector and contains the target center of mass.
//  Its x-axis is obtained by rotating
//  from the nose of the target through 'psiptx' and 'thtptx'.
//  Its y-axis remains in the target x,y-plane. 
//
//* Monte Carlo runs: for every sample run the intercept plane
//  will be oriented slightly different. Better, make two M.C. runs.
//  First run establishes the mean orientation angles 'psiptx','thtptx'
//  of the intercept plane. Then use 'mterm=2' to input these angles for
//  the actual M.C. run.
//
//030714 Created by Peter H Zipfel
//060509 Modified for SWEEP++, PZi
//150718 Added miss calculations in intercept plane, PZi
///////////////////////////////////////////////////////////////////////////////

void Missile::intercept(Packet *combus,int vehicle_slot,double int_step,const char *title)
{
	//local module-variables
	double hit_time(0);
	Matrix MISS(3,1);
	double miss(0);
	int mode(0);
	Matrix SBTP(3,1);
	Matrix TPL(3,3);

	//localizing module-variables
	//input data
	int mterm=missile[650].integer();
	double psiptx=missile[661].real();
	double thtptx=missile[662].real();
	//input from other modules
	double time=flat6[0].real();
	int halt=flat6[4].integer();
	int stop=flat6[5].integer();
	int lconv=flat6[6].integer();
	Matrix TBL=flat6[120].mat();
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	double dvbe=flat6[236].real();
	double hbe=flat6[239].real();
	double psivlx=flat6[240].real();
	double thtvlx=flat6[241].real();
	Matrix STEL=missile[2].vec(); 
	Matrix VTEL=missile[3].vec();
	int tgt_com_slot=missile[5].integer();
	int mprop=missile[50].integer();
	int trcond=missile[180].integer();
	int mseek=missile[200].integer();
	int mguid=missile[400].integer();
	int maut=missile[500].integer();
	//getting saved values
	int write=missile[651].integer();
	double time_m=missile[655].real();
	Matrix SBTLM=missile[656].vec();
	Matrix STMEL=missile[657].vec();
	Matrix SBMEL=missile[658].vec();
	//-------------------------------------------------------------------------
	//decoding guidance flag
    int guid_mid=mguid/10;
    int guid_term=(mguid%10);

	//trajectory mode flags
	mode=10000*mseek+1000*guid_mid+100*guid_term+10*maut+mprop;

	//true distance between blue missile and red target
	Matrix STBL=STEL-SBEL;
	Matrix STBB=TBL*STBL;
	double dbt=STBL.absolute();

	//Termination of trajectory if 'trcond' is set and 'stop=1' 
	if(trcond&&stop){

		//setting flag for termination
		lconv=4;

		//writing information to console
		//getting missile #
		string id_missl=combus[vehicle_slot].get_id();
		cout<<"\n"<<" *** Stopped Missile_"<<id_missl<<" with condition trcond = "<<trcond <<"   sim_time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s   altitude = "<<hbe<<" m     heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    

		//declaring missile 'dead'
		combus[vehicle_slot].set_status(0);
	}

	//Termination of trajectory if halt==1 
	if(halt){

		//setting flag for termination
		lconv=5;

		//writing information to console
		//getting missile #
		string id_missl=combus[vehicle_slot].get_id();
		cout<<"\n"<<" *** Halt of Missile_"<<id_missl<<"   sim_time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s   altitude = "<<hbe<<" m     heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    

		//declaring missile 'dead'
		combus[vehicle_slot].set_status(0);
	}
	//Ground impact
	double alt=-SBEL.get_loc(2,0);
	if((alt<=0)&&write){	
		write=0;

		//setting flag for termination
		lconv=3;

		//getting missile #
		string id_missl=combus[vehicle_slot].get_id();

		//writing information to console
//		cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
		cout<<"\n"<<" *** Ground impact of Missile_"<<id_missl<<" (or: #IND/#QNAN)  sim_time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    

		//declaring missile 'dead'
		combus[vehicle_slot].set_status(0);
	}

	//Sensor guidance
	if(mseek>=3)
	{
		//entering sphere of target influence of 100m 
		if(dbt<100)
		{		
			//unit LOS vector
			Matrix UTBL=STBL*(1/dbt);

			//differential velocity
			Matrix VTBEL=VTEL-VBEL;

			//closing speed on target, projected on LOS
			double closing_speed=UTBL^VTBEL;
			
			//between integration steps
			Matrix SBTL=STBL*(-1);

			//differential speed
			double diff_speed=VTBEL.absolute();

			//calculating head-on aspect angles of blue missile velocity vector wrt red missile velocity vector
			//heading and flight path angle of red missile
			Matrix POLAR=VTEL.pol_from_cart();
			double psivl=POLAR[1];
			double thtvl=POLAR[2];
			//T.M. of red missile wrt local level coordinates 
			Matrix TTL=mat2tr(psivl,thtvl);
			//differential velocity in red missile coordinates
			Matrix VTBET=TTL*VTBEL;
			//head-on aspect angles of blue missile wrt red missile velocity vectors 
			POLAR=VTBET.pol_from_cart();
			double az=POLAR[1];
			double el=POLAR[2];
			double aspazx=az*DEG;
			double aspelx=el*DEG;

			if((closing_speed>0)&&write){
			//intercept (closing speed becomes negative)
				write=0;
				if(mterm==0){
					//miss is closest approach between missile and target c.g.; obtained by linear interpolation
					//setting flag for termination
					lconv=2;

					Matrix SBBML=SBEL-SBMEL;
					Matrix STTML=STEL-STMEL;

					//intercept time at point of closest approach
					hit_time=time_m-int_step*((SBBML-STTML)^SBTLM)/(SBBML^SBBML);

					//getting missile # and target #
					string id_targ=combus[tgt_com_slot].get_id();
					string id_missl=combus[vehicle_slot].get_id();

					//absolute miss vector in geographic coordinates and miss magnitude
					double tau=hit_time-time_m;
					MISS=(SBBML-STTML)*(tau/int_step)+SBTLM;
					miss=MISS.absolute();
				
					//writing miss information to console
		//			cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
					cout<<"\n"<<" *** Intercept of Blue Missile_"<<id_missl<<" and Red Target_"<<id_targ<<" ***\n";
					cout<<"      miss magnitude = "<<miss<<" m    intercept sim_time = "<<hit_time<<" sec\n";
					cout<<"      north = "<<MISS.get_loc(0,0)<<" m      east = "<<MISS.get_loc(1,0)
									<<" m        down = "<<MISS.get_loc(2,0)<<" m\n";
					cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivlx<<" deg       gamma = "<<thtvlx<<" deg\n";    
					cout<<"      differential speed = "<<diff_speed<<" m/s\n";
					cout<<"      aspect of missile onto target aircraft: azimuth = "<<aspazx<<" deg   elevation = "<<aspelx<<" deg\n";
					cout<<"     Intercept condition 'trcond' = "<<trcond<<"\n\n";
				}
				if(mterm>0){				
					//miss vector in intercept plane
					//setting flag for termination
					lconv=2;

					Matrix SBBML=SBEL-SBMEL;
					Matrix STTML=STEL-STMEL;

					//intercept time at point of closest approach
					hit_time=time_m-int_step*((SBBML-STTML)^SBTLM)/(SBBML^SBBML);

					//getting missile # and target #
					string id_targ=combus[tgt_com_slot].get_id();
					string id_missl=combus[vehicle_slot].get_id();

					if(mterm==1){
						//for MC: every run will have lightly different aspect angles
						//yaw angle of relative vel.vec. in intercept plane
						Matrix VBTET=VTBET*(-1);
						POLAR=VBTET.pol_from_cart();
						psiptx=POLAR[1]*DEG;
						thtptx=POLAR[2]*DEG-90;
					}
					if(mterm>=1){
						//for MC: average aspect angles 'psiptx' and 'thtptx' are provided by 'input.asc'

						// T.M. of intercept plane wrt local level
						Matrix TPT=mat2tr(psiptx*RAD,thtptx*RAD);
						TPL=TPT*TTL;
						SBTP=TPL*SBTL;
						Matrix SBBMP=TPL*SBBML;
						Matrix STBMP=SBBMP-SBTP;
						double ww=STBMP[2]/SBBMP[2];
						//miss vector in intercept plane coordinates
						Matrix SBTP=SBBMP*ww-STBMP;
						MISS=SBTP;
						miss=MISS.absolute();

						//writing miss information to console
			//			cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
						cout<<"\n"<<" *** Intercept of Blue Missile_"<<id_missl<<" and Red Target_"<<id_targ<<" ***\n";
						cout<<"      miss in I-plane = "<<miss<<" m    intercept sim_time = "<<hit_time<<" sec\n";
						cout<<"      down = "<<MISS.get_loc(0,0)<<" m      right = "<<MISS.get_loc(1,0)<<" m\n";
						cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivlx<<" deg       gamma = "<<thtvlx<<" deg\n";    
						cout<<"      differential speed = "<<diff_speed<<" m/s\n";
						cout<<"      aspect of missile onto target aircraft: azimuth = "<<aspazx<<" deg   elevation = "<<aspelx<<" deg\n";
						cout<<"     Intercept condition 'trcond' = "<<trcond<<"\n\n";
					}
				}
				//declaring missile and target 'dead (=0)
				combus[vehicle_slot].set_status(0);
				combus[tgt_com_slot].set_status(0);

			}//end of closing speed change

			//save from previous cycle
			SBTLM=SBTL;
			STMEL=STEL;
			SBMEL=SBEL;
			time_m=time;

		}//end of sphere of influence
	}//end of sensor guidance
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	missile[651].gets(write);
	missile[653].gets(hit_time);
	missile[655].gets(time_m);
	missile[656].gets_vec(SBTLM);
	missile[657].gets_vec(STMEL);
	missile[658].gets_vec(SBMEL);
	//diagnostics
	flat6[6].gets(lconv);
	missile[652].gets(miss);
	missile[654].gets_vec(MISS);
	missile[659].gets(mode);
	missile[661].gets(psiptx);
	missile[662].gets(thtptx);
}
