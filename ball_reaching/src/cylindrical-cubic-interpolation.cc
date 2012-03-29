//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/factory.h>

#include "cylindrical-cubic-interpolation.hh"

namespace dynamicgraph {
  namespace sot {
    namespace reaching {
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CylindricalCubicInterpolation,
					 "CylindricalCubicInterpolationSE3");
      CylindricalCubicInterpolation::
      CylindricalCubicInterpolation (const std::string& name) :
	CubicInterpolation (name),
	localFrameSIN_ (NULL, "CylindricalCubicInterpolation("+name+
			")::input(MatrixHomo)::localFrame"),
	globalPos_ (3)
      {
	signalRegistration (localFrameSIN_);
	referenceSOUT_.setFunction (boost::bind (&CylindricalCubicInterpolation::
						 computeReference,
						 this, _1, _2));
	initialChestPose_.setIdentity ();
      }

      CylindricalCubicInterpolation::~CylindricalCubicInterpolation ()
      {
      }

      std::string CylindricalCubicInterpolation::getDocString () const
      {
	std::string doc =
	  "Perform a cubic interpolation in cylindrical coordinate in SE(3).\n"
	  "\n"
	  "  Initial pose is given by signal 'init', Target position is given"
	  " by signal\n"
	  "  'goal'. Interpolation is performed with zero velocities at start"
	  " and goal\n"
	  "  positions.\n"
	  "  Initial and goal poses are expressed in cylindrical coordinate in"
	  " frame given\n"
	  "  by signal 'localFrame' at beginning of motion.\n"
	  "\n"
	  "  This entity is aimed at performing reaching motion about the robot"
	  " chest.\n"
	  "\n";
	return doc;
      }

      sot::MatrixHomogeneous&
      CylindricalCubicInterpolation::computeReference (sot::MatrixHomogeneous&
						     reference, const int& time)
      {
	if (!motionStarted_) {
	  reference = initSIN_ (time);
	} else {
	  double t = (time - startTime_) * samplingPeriod_;
	  if (t <= duration_) {
	    cylindricalCoord_ = p0_ + (p1_ + (p2_ + p3_*t)*t)*t;
	    position_ (0) = cylindricalCoord_ (0) * cos (cylindricalCoord_ (1));
	    position_ (1) = cylindricalCoord_ (0) * sin (cylindricalCoord_ (1));
	    position_ (2) = cylindricalCoord_ (2);

	    initialChestPose_.multiply (position_, globalPos_);
	    reference (0,3) = globalPos_ (0);
	    reference (1,3) = globalPos_ (1);
	    reference (2,3) = globalPos_ (2);

	  } else {
	    motionStarted_ = false;
	    errorDot_.setZero ();
	    errorDotSOUT_.setConstant (errorDot_);
	    reference = goalSIN_ (time);
	  }
	}
	return reference;
      }

      void CylindricalCubicInterpolation::doStart (const double& duration)
      {
	// Check that sampling period has been initialized
	if (samplingPeriod_ <= 0)
	  throw ExceptionSignal (ExceptionSignal::NOT_INITIALIZED,
				 "CylindricalCubicSE3: samplingPeriod should"
				 " be positive. Are you sure you did\n"
				 "initialize it?");
	int time = initSIN_.getTime ();
	if (!motionStarted_) {
	  duration_ = duration;
	  startTime_ = referenceSOUT_.getTime ();
	  double T = duration;
	  // Express initial pose in cylindrical coordinate
	  initialChestPose_ = localFrameSIN_ (time);
	  inv_ = initialChestPose_.inverse ();
	  localPose_ = inv_ * initSIN_ (time);
	  double x = localPose_ (0,3);
	  double y = localPose_ (1,3);
	  double z = localPose_ (2,3);

	  // Initial position expressed in cylindrical coordinates
	  p0_ (0) = sqrt (x*x+y*y);
	  p0_ (1) = atan2 (y,x);
	  p0_ (2) = z;
	  // Initial velocity
	  p1_ (0) = 0.;
	  p1_ (1) = 0.;
	  p2_ (2) = 0.;
	  // Goal position expressed in cylindrical coordinates
	  localPose_ = inv_ * goalSIN_ (time);
	  x = localPose_ (0,3);
	  y = localPose_ (1,3);
	  z = localPose_ (2,3);
	  maal::boost::Vector P_T (3);
	  P_T (0) = sqrt (x*x+y*y);
	  P_T (1) = atan2 (y,x);
	  P_T (2) = z;
	  // Final velocity
	  maal::boost::Vector D_T (3); D_T.setZero ();
	  p2_ = (D_T + p1_*2)*(-1./T) + (P_T - p0_)*(3./(T*T));
	  p3_ = (P_T -p0_)*(-2/(T*T*T)) + (p1_ + D_T)*(1./(T*T));
	  motionStarted_ = true;
	}
      }
    } // reaching
  } // namespace sot
} // namespace dynamicgraph
