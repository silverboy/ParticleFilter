/*
 * FiltroParticulas.cpp
 *
 *  Created on: 15/07/2013
 *      Author: jplata
 */


#include "FiltroParticulas.h"

/** Update the m_particles, predicting the posterior of robot pose and map after a movement command.
*  This method has additional configuration parameters in "options".
*  Performs the update stage of the RBPF, using the sensed Sensorial Frame:
*
*   \param action This is a pointer to CActionCollection, containing the pose change the robot has been commanded.
*   \param observation This must be a pointer to a CSensoryFrame object, with robot sensed observations.
*
* \sa options
*/
void  CRangeBearingParticleFilter::prediction_and_update_pfStandardProposal(
		const mrpt::slam::CActionCollection	* action,
		const mrpt::slam::CSensoryFrame		* observation,
		const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	size_t i,N = m_particles.size();

	// Particle update:
	for (i=0;i<N;i++)
	{
		m_particles[i].d->x += randomGenerator.drawGaussian1D(mean,std);
		m_particles[i].d->y += randomGenerator.drawGaussian1D(mean,std);

	}

	CObservation2DRangeScanPtr obs = observation->getObservationByClass<CObservation2DRangeScan>();
	ASSERT_(obs);
	ASSERT_(obs->scan.size()==2);
	float obsX = obs->scan[0];
	float obsY = obs->scan[1];

	// Update weights
	for (i=0;i<N;i++)
	{
		float distance   = sqrt( square(m_particles[i].d->x - obsX) + square(m_particles[i].d->y - obsY));

		m_particles[i].log_w +=
			log( math::normalPDF( distance, 0, std ) );

	}

	// Resample is automatically performed by CParticleFilter when required.
}


void  CRangeBearingParticleFilter::initializeParticles(size_t  M,CPose2D initialPose)
{
	clearParticles();
	m_particles.resize(M);
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
		it->d = new CParticlePersonData();

	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
	{
		(*it).d->x  = randomGenerator.drawUniform( initialPose.x() - 0.25f, initialPose.x() + 0.25f );
		(*it).d->y  = randomGenerator.drawUniform( initialPose.y() - 0.25f, initialPose.y() + 0.25f );

		it->log_w	= 0;
	}

}

/** Computes the average velocity
  */
void CRangeBearingParticleFilter::getMean( double &x, double &y)
{
	double sumW=0;
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
		sumW+=exp( it->log_w );

	ASSERT_(sumW>0)

	x = y = 0;

	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
	{
		const double w = exp(it->log_w) / sumW;

		x += w * (*it).d->x;
		y += w * (*it).d->y;
	}
}

void CRangeBearingParticleFilter::setGaussianParams(double mean,double std){

	this->mean=mean;
	this->std=std;


}

Tracker::Tracker(){

	randomGenerator.randomize();

	PF_options.adaptiveSampleSize = false;
	PF_options.PF_algorithm = CParticleFilter::pfStandardProposal;
	PF_options.resamplingMethod = CParticleFilter::prSystematic;

	ASSERT_FILE_EXISTS_("../CONFIG_Measure.ini");

	CConfigFile config("../CONFIG_Measure.ini");

	double mean=config.read_double("PF","MEAN",0.1,false);
	double std=config.read_double("PF","STD",0.05,false);
	particles.setGaussianParams(mean,std);

	PF.m_options = PF_options;
}


void Tracker::inicializar(size_t numParticles,CPose2D initialPose){

	observacionAnterior=initialPose;
	particles.initializeParticles(numParticles,initialPose);

}


CPose2D Tracker::obtenerPosicionEstimada(CPose2D observacion,bool nuevaObservacion){

	if(nuevaObservacion){
		observacionAnterior=observacion;
	}

	// Process with PF:
	CSensoryFrame SF;
	CObservation2DRangeScanPtr obsRange = CObservation2DRangeScan::Create();
	obsRange->scan.resize(2);
	obsRange->scan[0] = observacionAnterior.x();
	obsRange->scan[1] = observacionAnterior.y();
	SF.insert( obsRange );  // memory freed by SF.

	PF.executeOn(particles, NULL,&SF);  // Process in the PF

	// Show PF state:
	cout << "Particle filter ESS: " << particles.ESS() << endl;

	double x,y;

	particles.getMean(x,y);

	CPose2D pose(x,y,0);

	return pose;

}

void Tracker::drawParticles(CDisplayWindowPlots *winPlot){

	size_t i,N = particles.m_particles.size();
	vector_float   parts_x(N),parts_y(N);
	for (i=0;i<N;i++)
	{
		parts_x[i] = particles.m_particles[i].d->x;
		parts_y[i] = particles.m_particles[i].d->y;
	}

	winPlot->plot( parts_x, parts_y, "b.2", "particles" );

}


