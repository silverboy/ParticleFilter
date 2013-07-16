/*
 * FiltroParticulas.h
 *
 *  Created on: 15/07/2013
 *      Author: jplata
 */

#ifndef FILTROPARTICULAS_H_
#define FILTROPARTICULAS_H_



#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <mrpt/obs.h>

#include <mrpt/bayes/CParticleFilter.h>

using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace std;



// ---------------------------------------------------------------
//		Implementation of the system models as a Particle Filter
// ---------------------------------------------------------------
struct CParticlePersonData
{
	double x,y; // Person position
};

class CRangeBearingParticleFilter : public mrpt::bayes::CParticleFilterCapable, public mrpt::bayes::CParticleFilterData<CParticlePersonData>
{
	// This uses CParticleFilterData to implement some methods required for CParticleFilterCapable:
	IMPLEMENT_PARTICLE_FILTER_CAPABLE(CParticlePersonData);

private:
	double mean;
	double std;
	double std_w;
	double dx;
	double dy;


public:

	 /** Update the m_particles, predicting the posterior of robot pose and map after a movement command.
	  *  This method has additional configuration parameters in "options".
	  *  Performs the update stage of the RBPF, using the sensed Sensorial Frame:
	  *
	  *   \param action This is a pointer to CActionCollection, containing the pose change the robot has been commanded.
	  *   \param observation This must be a pointer to a CSensoryFrame object, with robot sensed observations.
	  *
	  * \sa options
	  */
	void  prediction_and_update_pfStandardProposal(
		const mrpt::slam::CActionCollection	* action,
		const mrpt::slam::CSensoryFrame		* observation,
		const bayes::CParticleFilter::TParticleFilterOptions &PF_options );


	void initializeParticles(size_t  numParticles,CPose2D initialPose);

	/** Computes the average velocity & position
   	  */
	void getMean( double &x, double &y);

	void setGaussianParams(double mean,double std, double std_w);

	void setDisplacement(double dx, double dy);

};

class Tracker{

public:

	Tracker();
	void inicializar(size_t  numParticles,CPose2D initialPose);
	CPose2D obtenerPosicionEstimada(CPose2D observacion, bool nuevaObservacion);
	void drawParticles(CDisplayWindowPlots *plot);



private:

	CParticleFilter::TParticleFilterOptions	PF_options;
	CRangeBearingParticleFilter  particles;
	CParticleFilter	PF;
	CPose2D observacionAnterior;
};













#endif /* FILTROPARTICULAS_H_ */
