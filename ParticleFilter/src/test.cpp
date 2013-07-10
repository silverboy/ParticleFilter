/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
// ------------------------------------------------------
//  Refer to the description in the wiki:
//  http://www.mrpt.org/Kalman_Filters
// ------------------------------------------------------

#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <mrpt/obs.h>

#include <mrpt/bayes/CKalmanFilterCapable.h>

using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace std;

#define BEARING_SENSOR_NOISE_STD 	DEG2RAD(15.0f)
#define RANGE_SENSOR_NOISE_STD 		0.3f
#define DELTA_TIME                  	0.1f

#define VEHICLE_INITIAL_X			4.0f
#define VEHICLE_INITIAL_Y			4.0f
#define VEHICLE_INITIAL_V           1.0f
#define VEHICLE_INITIAL_W           DEG2RAD(20.0f)

#define TRANSITION_MODEL_STD_XY 	0.03f
#define TRANSITION_MODEL_STD_VXY 	0.20f

#define NUM_PARTICLES				2000

// Uncomment to save text files with grount truth vs. estimated states
//#define SAVE_GT_LOGS


// ---------------------------------------------------------------
//		Implementation of the system models as a Particle Filter
// ---------------------------------------------------------------
struct CParticleVehicleData
{
	float x,y, vx,vy; // Vehicle state (position & velocities)
};

class CRangeBearingParticleFilter : public mrpt::bayes::CParticleFilterCapable, public mrpt::bayes::CParticleFilterData<CParticleVehicleData>
{
	// This uses CParticleFilterData to implement some methods required for CParticleFilterCapable:
	IMPLEMENT_PARTICLE_FILTER_CAPABLE(CParticleVehicleData);

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


	void initializeParticles(size_t  numParticles);

	/** Computes the average velocity & position
   	  */
	void getMean( float &x, float &y, float &vx, float &vy );

};



// ------------------------------------------------------
//				TestBayesianTracking
// ------------------------------------------------------
void TestBayesianTracking()
{
	randomGenerator.randomize();

	CDisplayWindowPlots		winPF("Tracking - Particle Filter",450,400);


	winPF.setPos(480,10);

	winPF.axis(-2,20,-10,10);  winPF.axis_equal();


	// Create PF
	// ----------------------
	CParticleFilter::TParticleFilterOptions	PF_options;
	PF_options.adaptiveSampleSize = false;
	PF_options.PF_algorithm = CParticleFilter::pfStandardProposal;
	PF_options.resamplingMethod = CParticleFilter::prSystematic;

	CRangeBearingParticleFilter  particles;
	particles.initializeParticles(NUM_PARTICLES);
	CParticleFilter	PF;
	PF.m_options = PF_options;

#ifdef SAVE_GT_LOGS
	CFileOutputStream  fo_log_ekf("log_GT_vs_EKF.txt");
	fo_log_ekf.printf("%%%% GT_X  GT_Y  EKF_MEAN_X  EKF_MEAN_Y   EKF_STD_X   EKF_STD_Y\n");
#endif

	// Init. simulation:
	// -------------------------
	float x=VEHICLE_INITIAL_X,y=VEHICLE_INITIAL_Y,phi=DEG2RAD(-180),v=VEHICLE_INITIAL_V,w=VEHICLE_INITIAL_W;
	float  t=0;

	while (winPF.isOpen() && !mrpt::system::os::kbhit() )
	{
		// Update vehicle:
		x+=v*DELTA_TIME*(cos(phi)-sin(phi));
		y+=v*DELTA_TIME*(sin(phi)+cos(phi));
		phi+=w*DELTA_TIME;

		v+=1.0f*DELTA_TIME*cos(t);
		w-=0.1f*DELTA_TIME*sin(t);


		// Simulate noisy observation:
		float realBearing = atan2( y,x );
		float obsBearing = realBearing  + BEARING_SENSOR_NOISE_STD * randomGenerator.drawGaussian1D_normalized();
		printf("Real/Simulated bearing: %.03f / %.03f deg\n", RAD2DEG(realBearing), RAD2DEG(obsBearing) );

		float realRange = sqrt(square(x)+square(y));
		float obsRange = max(0.0, realRange  + RANGE_SENSOR_NOISE_STD * randomGenerator.drawGaussian1D_normalized() );
		printf("Real/Simulated range: %.03f / %.03f \n", realRange, obsRange );


		// Process with PF:
		CSensoryFrame SF;
		CObservationBearingRangePtr obsRangeBear = CObservationBearingRange::Create();
		obsRangeBear->sensedData.resize(1);
		obsRangeBear->sensedData[0].range = obsRange;
		obsRangeBear->sensedData[0].yaw   = obsBearing;
		SF.insert( obsRangeBear );  // memory freed by SF.

		PF.executeOn(particles, NULL,&SF);  // Process in the PF




		printf("Real: x:%.03f  y=%.03f heading=%.03f v=%.03f w=%.03f\n",x,y,phi,v,w);


		// Show PF state:
		cout << "Particle filter ESS: " << particles.ESS() << endl;


		// Draw PF state:
		{
			size_t i,N = particles.m_particles.size();
			vector_float   parts_x(N),parts_y(N);
			for (i=0;i<N;i++)
			{
				parts_x[i] = particles.m_particles[i].d->x;
				parts_y[i] = particles.m_particles[i].d->y;
			}

			winPF.plot( parts_x, parts_y, "b.2", "particles" );

			// Draw PF velocities:
			float avrg_x, avrg_y, avrg_vx,avrg_vy;

			particles.getMean(avrg_x, avrg_y, avrg_vx,avrg_vy);

			vector_float vx(2),vy(2);
			vx[0] = avrg_x;  vx[1] = vx[0] + avrg_vx * 1;
			vy[0] = avrg_y;  vy[1] = vy[0] + avrg_vy * 1;
			winPF.plot( vx,vy, "g-4", "velocityPF" );
		}

		// Draw GT:
		winPF.plot( vector_float(1,x), vector_float(1,y),"k.8","plot_GT");


		// Draw noisy observations:
		vector_float  obs_x(2),obs_y(2);
		obs_x[0] = obs_y[0] = 0;
		obs_x[1] = obsRange * cos( obsBearing );
		obs_y[1] = obsRange * sin( obsBearing );

		winPF.plot(obs_x,obs_y,"r", "plot_obs_ray");


		// Delay:
		mrpt::system::sleep((int)(DELTA_TIME*1000));
		t+=DELTA_TIME;
	}
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestBayesianTracking();
		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}




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

	// Transition model:
	for (i=0;i<N;i++)
	{
		m_particles[i].d->x += DELTA_TIME*m_particles[i].d->vx + TRANSITION_MODEL_STD_XY * randomGenerator.drawGaussian1D_normalized();
		m_particles[i].d->y += DELTA_TIME*m_particles[i].d->vy + TRANSITION_MODEL_STD_XY * randomGenerator.drawGaussian1D_normalized();

		m_particles[i].d->vx += TRANSITION_MODEL_STD_VXY * randomGenerator.drawGaussian1D_normalized();
		m_particles[i].d->vy += TRANSITION_MODEL_STD_VXY * randomGenerator.drawGaussian1D_normalized();
	}

	CObservationBearingRangePtr obs = observation->getObservationByClass<CObservationBearingRange>();
	ASSERT_(obs);
	ASSERT_(obs->sensedData.size()==1);
	float obsRange = obs->sensedData[0].range;
	float obsBearing = obs->sensedData[0].yaw;

	// Update weights
	for (i=0;i<N;i++)
	{
		float predicted_range   = sqrt( square(m_particles[i].d->x)+square(m_particles[i].d->y));
		float predicted_bearing = atan2( m_particles[i].d->y, m_particles[i].d->x );

		m_particles[i].log_w +=
			log( math::normalPDF( predicted_range-obsRange, 0, RANGE_SENSOR_NOISE_STD ) ) +
			log( math::normalPDF( math::wrapToPi( predicted_bearing-obsBearing), 0, BEARING_SENSOR_NOISE_STD ) );
	}

	// Resample is automatically performed by CParticleFilter when required.
}


void  CRangeBearingParticleFilter::initializeParticles(size_t  M)
{
	clearParticles();
	m_particles.resize(M);
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
		it->d = new CParticleVehicleData();

	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
	{
		(*it).d->x  = randomGenerator.drawUniform( VEHICLE_INITIAL_X - 2.0f, VEHICLE_INITIAL_X + 2.0f );
		(*it).d->y  = randomGenerator.drawUniform( VEHICLE_INITIAL_Y - 2.0f, VEHICLE_INITIAL_Y + 2.0f );

		(*it).d->vx = randomGenerator.drawGaussian1D( -VEHICLE_INITIAL_V, 0.2f );
		(*it).d->vy = randomGenerator.drawGaussian1D( 0, 0.2f );

		it->log_w	= 0;
	}

}

/** Computes the average velocity
  */
void CRangeBearingParticleFilter::getMean( float &x, float &y, float &vx, float &vy )
{
	double sumW=0;
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
		sumW+=exp( it->log_w );

	ASSERT_(sumW>0)

	x = y = vx = vy = 0;

	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
	{
		const double w = exp(it->log_w) / sumW;

		x += (float)w * (*it).d->x;
		y += (float)w * (*it).d->y;
		vx+= (float)w * (*it).d->vx;
		vy+= (float)w * (*it).d->vy;
	}
}

