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
		m_particles[i].d->x +=dx + (2*(rand()%2)-1)*randomGenerator.drawGaussian1D(mean,std);
		m_particles[i].d->y +=dy + (2*(rand()%2)-1)*randomGenerator.drawGaussian1D(mean,std);

		//m_particles[i].d->x +=dx + randomGenerator.drawGaussian1D(0,std);
		//m_particles[i].d->y +=dy + randomGenerator.drawGaussian1D(0,std);
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
			log( math::normalPDF( distance, 0, std_w ) );

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

void CRangeBearingParticleFilter::setGaussianParams(double mean,double std, double std_w){

	this->mean=mean;
	this->std=std;
	this->std_w=std_w;


}

void CRangeBearingParticleFilter::setDisplacement(double dx, double dy){

	this->dx=dx;
	this->dy=dy;


}

Tracker::Tracker(){

	randomGenerator.randomize();

	PF_options=CParticleFilter::TParticleFilterOptions();
	particles=CRangeBearingParticleFilter();
	PF=CParticleFilter();

	PF_options.adaptiveSampleSize = false;
	PF_options.PF_algorithm = CParticleFilter::pfStandardProposal;
	PF_options.resamplingMethod = CParticleFilter::prSystematic;

	ASSERT_FILE_EXISTS_("../CONFIG_Measure.ini");

	CConfigFile config("../CONFIG_Measure.ini");


	double mean=config.read_double("PF","MEAN",0.1,false);
	double std=config.read_double("PF","STD",0.05,false);
	double std_w=config.read_double("PF","STD_W",0.2,false);
	particles.setGaussianParams(mean,std,std_w);
	particles.setDisplacement(0,0);

	PF.m_options = PF_options;
}


void Tracker::inicializar(size_t numParticles,CPose2D initialPose){

	observacionAnterior=initialPose;
	particles.initializeParticles(numParticles,initialPose);

}


CPose2D Tracker::obtenerPosicionEstimada(CPose2D observacion,bool nuevaObservacion){

	if(nuevaObservacion){
		CPose2D d=observacion-observacionAnterior;
		particles.setDisplacement(d.x(),d.y());

		observacionAnterior=observacion;
	}
	else{
		particles.setDisplacement(0,0);

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

	winPlot->plot( parts_x, parts_y, "b.1", "particles" );
}

void Tracker::setGaussianParams(double mean,double std,double std_w){
	particles.setGaussianParams(mean,std,std_w);
}


