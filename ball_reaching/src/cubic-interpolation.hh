// 
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#ifndef SOT_REACHING_HH
# define SOT_REACHING_HH

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>
# include <sot/core/matrix-homogeneous.hh>

# include <jrl/mal/matrixabstractlayer.hh>

namespace dynamicgraph {
  namespace sot {
    namespace reaching {
      class CubicInterpolation : public Entity
      {
	DYNAMIC_GRAPH_ENTITY_DECL();
      public:
	CubicInterpolation (const std::string& name);
	/// Start tracking
	void start (const double& duration);
	/// Documentation
	virtual std::string getDocString () const;
	/// Set sampling period of control discretization
	void setSamplingPeriod (const double& period);
      private:
	dynamicgraph::Signal < MatrixHomogeneous, int > referenceSOUT_;
	dynamicgraph::Signal < maal::boost::Vector, int > errorDotSOUT_;
	dynamicgraph::SignalPtr < MatrixHomogeneous, int > initSIN_;
	dynamicgraph::SignalPtr < MatrixHomogeneous, int > goalSIN_;
	
	MatrixHomogeneous& computeReference (MatrixHomogeneous& reference,
						    const int& time);

	int startTime_;
	double samplingPeriod_;
	double duration_;
	bool motionStarted_;

	maal::boost::Vector p0_;
	maal::boost::Vector p1_;
	maal::boost::Vector p2_;
	maal::boost::Vector p3_;

	maal::boost::Vector position_;
	maal::boost::Vector errorDot_;
      }; // class CubicInterpolation
    } // reaching
  } // namespace sot
} // namespace dynamicgraph

#endif // SOT_REACHING_HH
