//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/factory.h>

#include "cubic-interpolation.hh"

namespace dynamicgraph {
  namespace sot {
    namespace reaching {
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CubicInterpolation,
					 "CubicInterpolationSE3");
      CubicInterpolation::CubicInterpolation (const std::string& name) :
	Entity (name),
	referenceSOUT_ ("CubicInterpolation("+name+
			")::output(vector)::reference"),
	errorDotSOUT_ ("CubicInterpolation("+name+
			")::output(vector)::errorDot"),
	initSIN_ (NULL, "CubicInterpolation("+name+")::input(vector)::init"),
	goalSIN_ (NULL, "CubicInterpolation("+name+")::input(vector)::goal"),
	startTime_ (0), samplingPeriod_ (0.), motionStarted_ (false),
	p0_ (3), p1_ (3), p2_ (3), p3_ (3), position_ (3), errorDot_ (3)
      {
	signalRegistration (referenceSOUT_);
	signalRegistration (errorDotSOUT_);
	signalRegistration (initSIN_);
	signalRegistration (goalSIN_);
	referenceSOUT_.setFunction (boost::bind
				    (&CubicInterpolation::computeReference,
				     this, _1, _2));
	errorDot_.setZero ();
	errorDotSOUT_.setConstant (errorDot_);

	std::string docstring;
	docstring =
	  "\n"
	  "    Set sampling period of control discretization.\n"
	  "\n"
	  "    Input:\n"
	  "      - a floating point value.\n"
	  "\n";
	addCommand ("setSamplingPeriod",
		    new command::Setter <CubicInterpolation, double>
		    (*this, &CubicInterpolation::setSamplingPeriod, docstring));
	docstring =
	  "\n"
	  "    Start tracking.\n"
	  "\n"
	  "    Input\n"
	  "      - duration of the motion.\n"
	  "\n"
	  "\n  Read init and goal signals, compute reference tracjectory and start\n"
	  "tracking.\n";
	addCommand ("start",
		    new command::Setter <CubicInterpolation, double>
		    (*this, &CubicInterpolation::start, docstring));
      }

      std::string CubicInterpolation::getDocString () const
      {
	std::string doc =
	  "Perform a cubic interpolation in SE(3) between two poses.\n"
	  "\n"
	  "  Initial pose is given by signal 'init', Target position is given"
	  " by signal\n"
	  "  'goal'. Interpolation is performed with zero velocities at start"
	  " and goal\n"
	  "  positions.\n";
	return doc;
      }

      sot::MatrixHomogeneous&
      CubicInterpolation::computeReference (sot::MatrixHomogeneous&
					    reference, const int& time)
      {
	if (!motionStarted_) {
	  reference = initSIN_ (time);
	} else {
	  double t = (time - startTime_) * samplingPeriod_;
	  if (t <= duration_) {
	    position_ = p0_ + (p1_ + (p2_ + p3_*t)*t)*t;
	    reference (0,3) = position_ (0);
	    reference (1,3) = position_ (1);
	    reference (2,3) = position_ (2);

	    errorDot_ = p1_ + (p2_*2 + p3_*(3*t))*t;
	    errorDotSOUT_.setConstant (errorDot_);
	  } else {
	    motionStarted_ = false;
	    errorDot_.setZero ();
	    errorDotSOUT_.setConstant (errorDot_);
	    reference = goalSIN_ (time);
	  }
	}
	return reference;
      }

      void CubicInterpolation::setSamplingPeriod (const double& period)
      {
	samplingPeriod_ = period;
      }

      void CubicInterpolation::start (const double& duration)
      {
	// Check that sampling period has been initialized
	if (samplingPeriod_ <= 0)
	  throw ExceptionSignal (ExceptionSignal::NOT_INITIALIZED,
				 "CubicInterpolationSE3: samplingPeriod should"
				 " be positive. Are you sure you did\n"
				 "initialize it?");
	int time = initSIN_.getTime ();
	if (!motionStarted_) {
	  duration_ = duration;
	  startTime_ = referenceSOUT_.getTime ();
	  double T = duration;
	  // Initial position
	  p0_ (0) = initSIN_ (time) (0,3);
	  p0_ (1) = initSIN_ (time) (1,3);
	  p0_ (2) = initSIN_ (time) (2,3);
	  // Initial velocity
	  p1_ (0) = 0.;
	  p1_ (1) = 0.;
	  p2_ (2) = 0.;
	  // Goal position
	  maal::boost::Vector P_T (3);
	  P_T (0) = goalSIN_ (time) (0,3);
	  P_T (1) = goalSIN_ (time) (1,3);
	  P_T (2) = goalSIN_ (time) (2,3);
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
