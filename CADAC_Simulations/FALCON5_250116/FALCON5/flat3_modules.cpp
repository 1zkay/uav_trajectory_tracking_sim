///////////////////////////////////////////////////////////////////////////////
//FILE: 'flat3_modules.cpp'
//
//Contains all modules of class 'Flat3'
//							kinematics()  flat3[0-9]
//							environment() flat3[10-19]
//							newton()	  flat3[20-39]
//
//020114 Created by Peter H Zipfel
//030319 Upgraded to SM Item32, PZi
//130725 Building AIM5, PZi
//231117 Used without change for FALCON5, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Defining of 'kinematics' module-variables
//Member function of class 'Flat3'
//Module-variable locations are assigned to flat3[0-9]
//
//Initializing the module-variables
//		
//051018 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat3::def_kinematics()
{
	//defining module-variables
	flat3[0].init("time",0,"Vehicle time since launch - s","kinematics","exec","scrn,plot");
	flat3[1].init("event_time",0,"Vehicle time in event - s","kinematics","exec","");
}
///////////////////////////////////////////////////////////////////////////////
//Initial calculations of 'kinematics' module 
//Member function of class 'Flat3'
// 
//Timing 
//
//051018 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat3::init_kinematics(double sim_time)
{
	//local module-variables
	double time(0);

	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//-------------------------------------------------------------------------
	//loading module-variables
	flat3[0].gets(time);
}
///////////////////////////////////////////////////////////////////////////////
//'kinematic' module
//Member function of class 'Flat3'
//
//Timing functions
//
//051018 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat3::kinematics(double sim_time,double event_time)
{
	//local module-variables
	double time(0);

	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//-------------------------------------------------------------------------
	//loading module-variables
	flat3[0].gets(time);
	flat3[1].gets(event_time);
}
///////////////////////////////////////////////////////////////////////////////
//Defining of 'environmental' module-variables
//Member function of class 'Flat3'
//Module-variable locations are assigned to flat3[10-19]
//
//Initializing the module-variables
//		
//020114 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat3::def_environment()
{
	//defining module-variables
	flat3[11].init("grav",0,"Gravitational acceleration - m/s^2","environment","out","");
	flat3[12].init("rho",0,"Air density - kg/m^3","environment","out","");
	flat3[13].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","scrn,plot");
	flat3[14].init("mach",0,"Mach number - ND","environment","out","scrn,plot");
	flat3[15].init("vsound",0,"Speed of sound - m/s","environment","diag","");
	flat3[16].init("press",0,"Atmospheric pressure - Pa","environment","out","");
}
///////////////////////////////////////////////////////////////////////////////
//'environment' module
//Member function of class 'Flat3' 
//
//US 1976 Standard Atmosphere
//
//020114 Created by Peter H Zipfel
//030319 Upgraded to US76 atmosphere, PZi
///////////////////////////////////////////////////////////////////////////////
void Flat3::environment()
{	
	//local variables
	double tempk(0);
	
	//localized module-variables	
	double grav(0);
	double rho(0); 
	double pdynmc(0);
	double mach(0);
	double vsound(0);
	double alt(0);
	double press(0);

	//localizing module-variables
	double dvbe=flat3[25].real();
	Matrix SBEL=flat3[26].vec();
	//-------------------------------------------------------------------------
	//Altitude above flat Earth
	alt=-SBEL.get_loc(2,0);

	//Newtonian gravitational acceleration
	grav=G*EARTH_MASS/pow((REARTH+alt),2);

	//US 1976 Standard Atmosphere
	atmosphere76(rho,press,tempk, alt);

	vsound=sqrt(1.4*R*tempk);
	mach=fabs(dvbe/vsound);
	pdynmc=0.5*rho*pow(dvbe,2);
	//-------------------------------------------------------------------------
	//loading module-variables
	flat3[11].gets(grav);
	flat3[12].gets(rho);
	flat3[13].gets(pdynmc);
	flat3[14].gets(mach);
	flat3[15].gets(vsound);
	flat3[16].gets(press);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'newton' module-variables 
//Member function of class 'Flat3'
//Module-variable locations are assigned to flat3[20-39]
// 
//Initializing variables for the Newton Module.
//
//020114 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat3::def_newton()
{
	//Definition of module-variables
	flat3[22].init("TBL",0,0,0,0,0,0,0,0,0,"TM of vehicle wrt loc lev coor","newton","out","");
	flat3[23].init("TBV",0,0,0,0,0,0,0,0,0,"TM of vehicle wrt velocity coor","newton","diag","");
	flat3[24].init("TVL",0,0,0,0,0,0,0,0,0,"TM of vel coor wrt loc lev coor ","newton","diag","");
	flat3[25].init("dvbe",0,"Vehicle speed - m/s","newton","init/out","scrn");
	flat3[26].init("SBEL",0,0,0,"Vehicle position wrt point E - m","newton","state","plot");
	flat3[27].init("VBEL",0,0,0,"Vehicle velocity wrt Earth - m/s","newton","state","");
	flat3[28].init("ABEL",0,0,0,"Vehicle acceleration  wrt Earth - m/s^2 ","newton","state","");
	flat3[29].init("psivlx",0,"Vehicle heading angle - deg","newton","init/diag","scrn,plot");
	flat3[30].init("thtvlx",0,"Vehicle flight path angle - deg","newton","init/diag","scrn,plot");
	flat3[31].init("sbel1",0,"Vehicle initial north position - m","newton","init","");
	flat3[32].init("sbel2",0,"Vehicle initial east position - m","newton","init","");
	flat3[33].init("sbel3",0,"Vehicle initial down position - m","newton","init","");
	flat3[34].init("psivl",0,"Vehicle heading angle - rad","newton","out","");
	flat3[35].init("thtvl",0,"Vehicle flight path angle - rad","newton","out","");
	flat3[36].init("alt",0,"Vehicle altitude - m","newton","out","scrn,plot");
}
///////////////////////////////////////////////////////////////////////////////
//Initial calculations of 'newton' module 
//Member function of class 'Flat3'
// 
//Initial calculations.
//
//020114 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat3::init_newton()
{
	//local module-variables
	Matrix TBL(3,3);
	Matrix TBV(3,3);
	Matrix TVL(3,3);

	//localizing module-variables
	double phiavout=flat3[21].real();
	double dvbe=flat3[25].real();
	Matrix SBEL=flat3[26].vec();
	Matrix VBEL=flat3[27].vec();
	double psivlx=flat3[29].real();
	double thtvlx=flat3[30].real();
	double sbel1=flat3[31].real();
	double sbel2=flat3[32].real();
	double sbel3=flat3[33].real();
	//-------------------------------------------------------------------------
	//conversion
	double psivl=psivlx*RAD;
	double thtvl=thtvlx*RAD;

	//calculating initial vehicle position in local level coord
	VBEL.cart_from_pol(dvbe,psivl,thtvl);

	//initializing the T.M. TBL
	TVL=mat2tr(psivl,thtvl);
	//initializing the T.M. TBV
	TBV.identity();
	double cphi=cos(phiavout);
	double sphi=sin(phiavout);
	TBV.assign_loc(1,1,cphi);
	TBV.assign_loc(2,2,cphi);
	TBV.assign_loc(1,2,sphi);
	TBV.assign_loc(2,1,-sphi);
	//calculating the T.M. TBL of aircraft body ares wrt local level axes
	TBL=TBV*TVL;

	//defining initial postion vector
	SBEL.build_vec3(sbel1,sbel2,sbel3);
	//-------------------------------------------------------------------------
	//loading module-variables
	flat3[22].gets_mat(TBL);
	flat3[23].gets_mat(TBV);
	flat3[24].gets_mat(TVL);
	flat3[26].gets_vec(SBEL);
	flat3[27].gets_vec(VBEL);
}
///////////////////////////////////////////////////////////////////////////////
//'newton' module
//Member function of class 'Flat3'
//
//Solving the translational equations of motion using Newton's 2nd Law
//
//020114 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat3::newton(double int_step)
{
	//local variables
	Matrix TBV(3,3);
	Matrix GRAVL(3,1);

	//local module-variables
	double dvbe(0);
	double psivl(0),thtvl(0);
	double psivlx(0),thtvlx(0);
	Matrix TVL(3,3);
	double alt(0);

	//localizing module-variables
	//input from other modules
	Matrix FSPV=flat3[10].vec();
	double grav=flat3[11].real();
	double phiavout=flat3[21].real();
	Matrix TBL=flat3[22].mat();
	//state variables
	Matrix SBEL=flat3[26].vec();
	Matrix VBEL=flat3[27].vec();
	Matrix ABEL=flat3[28].vec();
	//-------------------------------------------------------------------------
	//building gravitational vector in local level coord
	GRAVL.build_vec3(0,0,grav);

	//integrating state variables
	Matrix TLB=TBL.trans();
	Matrix NEXT_ACC=TLB*FSPV+GRAVL;
	Matrix NEXT_VEL=integrate(NEXT_ACC,ABEL,VBEL,int_step);
	SBEL=integrate(NEXT_VEL,VBEL,SBEL,int_step);
	ABEL=NEXT_ACC;
	VBEL=NEXT_VEL;

	//calculating the T.M. TBL
	Matrix POLAR=VBEL.pol_from_cart();
	dvbe=POLAR.get_loc(0,0);
	psivl=POLAR.get_loc(1,0);
	thtvl=POLAR.get_loc(2,0);
	TVL=mat2tr(psivl,thtvl);

	//calculating the T.M. TBV
	TBV.identity();
	double cphi=cos(phiavout);
	double sphi=sin(phiavout);
	TBV.assign_loc(1,1,cphi);
	TBV.assign_loc(2,2,cphi);
	TBV.assign_loc(1,2,sphi);
	TBV.assign_loc(2,1,-sphi);

	//calculating the T.M. TBL of aircraft body ares wrt local level axes
	TBL=TBV*TVL;

	//diagnostic output
	psivlx=psivl*DEG;
	thtvlx=thtvl*DEG;
	alt=-SBEL[2];
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	flat3[26].gets_vec(SBEL);
	flat3[27].gets_vec(VBEL);
	flat3[28].gets_vec(ABEL);
	//output to other modules
	flat3[22].gets_mat(TBL);
	flat3[24].gets_mat(TVL);
	flat3[25].gets(dvbe);
	flat3[34].gets(psivl);
	flat3[35].gets(thtvl);
	//diagnostics
	flat3[29].gets(psivlx);
	flat3[30].gets(thtvlx);
	flat3[36].gets(alt);
}
